/*
Copyright 2025 Manifold Tech Ltd.(www.manifoldtech.com.co)
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "depth_image_ros2_node.hpp"
#include <functional>

// 构造函数：初始化深度图节点，加载相机参数并创建点云转深度图转换器
DepthImageRos2Node::DepthImageRos2Node(const rclcpp::NodeOptions & options)
    : Node("depth_image_ros2_node", options)
{
    PointCloudToDepthConverter::CameraParams camera_params = loadCameraParams();

    depth_converter_ = std::make_unique<PointCloudToDepthConverter>(camera_params);
    
    cloud_raw_topic_ = this->declare_parameter<std::string>("cloud_raw_topic", "/odin1/cloud_raw");
    color_raw_topic_ = this->declare_parameter<std::string>("color_raw_topic", "/odin1/image");
    depth_image_topic_ = this->declare_parameter<std::string>("depth_image_topic", "/odin1/depth_img_competetion");
    depth_cloud_topic_ = this->declare_parameter<std::string>("depth_cloud_topic", "/odin1/depth_img_competetion_cloud");
    // ApproximateTime 本身已经负责做“近似同步”。
    // 这里把默认队列深度恢复到更稳妥的 10，避免 image/cloud 任一侧存在轻微抖动时，
    // 由于缓存过浅导致本来可以配上的数据直接被冲掉。
    sync_queue_size_ = this->declare_parameter<int>("sync_queue_size", 10);
    // 额外的手工时间差过滤默认关闭。
    // 实测双重过滤容易把 ApproximateTime 已经匹配好的 pair 再次丢掉，表现为 depth topic 无数据。
    // 如确实需要更严苛的配对窗口，可显式传入 >0 的毫秒值重新启用。
    sync_tolerance_ms_ = this->declare_parameter<int>("sync_tolerance_ms", 0);
    publish_depth_cloud_ = this->declare_parameter<bool>("publish_depth_cloud", true);
    depth_postprocess_ = this->declare_parameter<bool>("depth_postprocess", true);

    RCLCPP_INFO_STREAM(this->get_logger(), 
                       "\n  cloud_raw_topic: " << cloud_raw_topic_
                       << "\n  color_raw_topic: " << color_raw_topic_
                       << "\n  depth_image_topic: " << depth_image_topic_
                       << "\n  depth_cloud_topic: " << depth_cloud_topic_
                       << "\n  sync_queue_size: " << sync_queue_size_
                       << "\n  sync_tolerance_ms: " << sync_tolerance_ms_
                       << "\n  publish_depth_cloud: " << publish_depth_cloud_
                       << "\n  depth_postprocess: " << depth_postprocess_);
}

// 订阅点云和图像话题，创建时间同步器，初始化深度图和点云发布器
void DepthImageRos2Node::initialize()
{
    cloud_sub_.subscribe(this, cloud_raw_topic_);
    color_sub_.subscribe(this, color_raw_topic_);

    sync_ = std::make_shared<Sync>(MySyncPolicy(sync_queue_size_), cloud_sub_, color_sub_);
    sync_->registerCallback(std::bind(&DepthImageRos2Node::syncCallback, this, 
                                     std::placeholders::_1, std::placeholders::_2));

    depth_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_image_topic_, 1);
    if (publish_depth_cloud_) {
        depth_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(depth_cloud_topic_, 1);
    }
    worker_thread_ = std::thread(&DepthImageRos2Node::workerLoop, this);

    RCLCPP_INFO(this->get_logger(), "DepthImageRos2Node initialized successfully");
}

DepthImageRos2Node::~DepthImageRos2Node()
{
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        stop_worker_ = true;
    }
    sync_cv_.notify_all();
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
}

// 从 ROS2 参数服务器加载相机内参、畸变系数和相机-激光雷达外参矩阵
PointCloudToDepthConverter::CameraParams DepthImageRos2Node::loadCameraParams()
{
    PointCloudToDepthConverter::CameraParams params;

    params.image_width = this->declare_parameter<int>("cam_0.image_width", 1600);
    params.image_height = this->declare_parameter<int>("cam_0.image_height", 1296);
    params.A11 = this->declare_parameter<double>("cam_0.A11", 0.0);
    params.A12 = this->declare_parameter<double>("cam_0.A12", 0.0);
    params.A22 = this->declare_parameter<double>("cam_0.A22", 0.0);
    params.u0 = this->declare_parameter<double>("cam_0.u0", 0.0);
    params.v0 = this->declare_parameter<double>("cam_0.v0", 0.0);

    params.k2 = this->declare_parameter<double>("cam_0.k2", 0.0);
    params.k3 = this->declare_parameter<double>("cam_0.k3", 0.0);
    params.k4 = this->declare_parameter<double>("cam_0.k4", 0.0);
    params.k5 = this->declare_parameter<double>("cam_0.k5", 0.0);
    params.k6 = this->declare_parameter<double>("cam_0.k6", 0.0);
    params.k7 = this->declare_parameter<double>("cam_0.k7", 0.0);

    params.scale = this->declare_parameter<double>("scale", 7.0);
    params.point_sampling_rate = this->declare_parameter<int>("point_sampling_rate", 5);

    std::vector<double> Tcl_vec_param = this->declare_parameter<std::vector<double>>("Tcl_0", std::vector<double>(16, 0.0));
    if (Tcl_vec_param.size() == 16)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                params.Tcl(i, j) = Tcl_vec_param[i * 4 + j];
            }
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Tcl_0 param missing or invalid, colored reproject cloud disabled.");
        rclcpp::shutdown();
    }

    if (params.A11 < 1e-6 || params.A22 < 1e-6 || params.u0 < 1e-6 || params.v0 < 1e-6)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid camera intrinsics A11 or A22");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Camera intrinsics:");
    RCLCPP_INFO(this->get_logger(), "Image size: %dx%d", params.image_width, params.image_height);
    RCLCPP_INFO(this->get_logger(), "Intrinsics: A11=%f A12=%f A22=%f u0=%f v0=%f",
             params.A11, params.A12, params.A22, params.u0, params.v0);
    RCLCPP_INFO(this->get_logger(), "Distortions: k2=%f k3=%f k4=%f k5=%f k6=%f k7=%f",
             params.k2, params.k3, params.k4, params.k5, params.k6, params.k7);
    RCLCPP_INFO(this->get_logger(), "Scale: %f, Point sampling rate: %d", params.scale, params.point_sampling_rate);
    RCLCPP_INFO_STREAM(this->get_logger(), "Extrinsics (Tcl):\n" << params.Tcl);

    return params;
}

// 时间同步回调：接收点云和 RGB 图像，调用深度图转换器生成并发布深度图和彩色点云
void DepthImageRos2Node::syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
                                     const sensor_msgs::msg::Image::ConstSharedPtr color_msg)
{
    // latest-only：每次同步回调只覆盖最近一组数据，真正的深度补全交给后台线程串行执行。
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        pending_cloud_msg_ = cloud_msg;
        pending_color_msg_ = color_msg;
    }
    sync_cv_.notify_one();
}

void DepthImageRos2Node::workerLoop()
{
    while (rclcpp::ok()) {
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg;
        sensor_msgs::msg::Image::ConstSharedPtr color_msg;

        {
            std::unique_lock<std::mutex> lock(sync_mutex_);
            sync_cv_.wait(lock, [&]() {
                return stop_worker_ || (pending_cloud_msg_ && pending_color_msg_);
            });
            if (stop_worker_) {
                return;
            }
            cloud_msg = pending_cloud_msg_;
            color_msg = pending_color_msg_;
            pending_cloud_msg_.reset();
            pending_color_msg_.reset();
        }

        if (sync_tolerance_ms_ > 0) {
            const int64_t time_delta_ns =
                static_cast<int64_t>(rclcpp::Time(cloud_msg->header.stamp).nanoseconds()) -
                static_cast<int64_t>(rclcpp::Time(color_msg->header.stamp).nanoseconds());
            if (std::abs(time_delta_ns) > static_cast<int64_t>(sync_tolerance_ms_) * 1000000LL) {
                continue;
            }
        }

        const bool want_depth_image =
            depth_image_pub_ && depth_image_pub_->get_subscription_count() > 0;
        const bool want_depth_cloud =
            publish_depth_cloud_ &&
            depth_cloud_pub_ &&
            depth_cloud_pub_->get_subscription_count() > 0;
        if (!want_depth_image && !want_depth_cloud) {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);
        if (cloud.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty point cloud received");
            continue;
        }

        cv::Mat img_raw;
        try
        {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(color_msg, "bgr8");
            img_raw = cv_ptr->image;
            if (img_raw.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Failed to decode compressed image");
                continue;
            }
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge: %s", e.what());
            continue;
        }

        auto result = depth_converter_->processCloudAndImage(
            cloud, img_raw, depth_postprocess_, want_depth_cloud);

        if (!result.success)
        {
            RCLCPP_WARN(this->get_logger(), "Data processing failed: %s", result.error_message.c_str());
            continue;
        }

        if (want_depth_image) {
            publishDepthImage(result.depth_image, cloud_msg->header);
        }
        if (want_depth_cloud) {
            publishDepthCloud(result.colored_cloud, cloud_msg->header);
        }
    }
}

// 将 OpenCV 深度图像转换为 ROS2 Image 消息并发布
void DepthImageRos2Node::publishDepthImage(const cv::Mat &img,
                                          const std_msgs::msg::Header &header,
                                          const std::string &encoding)
{
    if (!depth_image_pub_) {
        return;
    }
    sensor_msgs::msg::Image::SharedPtr depth_msg = cv_bridge::CvImage(header, encoding, img).toImageMsg();
    depth_image_pub_->publish(*depth_msg);
}

// 将带颜色的 PCL 点云转换为 ROS2 PointCloud2 消息并发布
void DepthImageRos2Node::publishDepthCloud(const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud,
                                          const std_msgs::msg::Header &header)
{
    if (!depth_cloud_pub_) {
        return;
    }
    if (!colored_cloud.points.empty())
    {
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(colored_cloud, cloud_msg);
        cloud_msg.header = header;
        depth_cloud_pub_->publish(cloud_msg);
    }
}
