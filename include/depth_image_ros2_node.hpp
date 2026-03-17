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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Dense>

#include "pointcloud_depth_converter.hpp"

#include <string>
#include <memory>

// 深度图生成 ROS2 节点：订阅原始点云和 RGB 图像，融合生成稠密深度图和彩色点云
class DepthImageRos2Node : public rclcpp::Node
{
public:
    // 构造函数：加载相机参数，初始化深度图转换器
    explicit DepthImageRos2Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    // 初始化：订阅话题，创建时间同步器和发布器（必须在构造后调用）
    void initialize();

private:
    std::string cloud_raw_topic_;
    std::string color_compressed_topic_;
    std::string color_raw_topic_;
    std::string depth_image_topic_;
    std::string depth_cloud_topic_;

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> color_compressed_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, 
        // sensor_msgs::msg::CompressedImage,
        sensor_msgs::msg::Image> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Publisher depth_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_cloud_pub_;

    std::unique_ptr<PointCloudToDepthConverter> depth_converter_;


    // 从 ROS2 参数服务器加载相机内参、畸变系数和外参矩阵
    PointCloudToDepthConverter::CameraParams loadCameraParams();

    // 时间近似同步回调：接收点云和 RGB 图像，生成深度图和彩色点云并发布
    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
                      // const sensor_msgs::msg::CompressedImage::ConstSharedPtr image_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr color_msg);

    // 将深度图（32FC1）转换为 ROS2 Image 消息并发布
    void publishDepthImage(const cv::Mat &img,
                           const std_msgs::msg::Header &header,
                           const std::string &encoding = "32FC1");

    // 将彩色 PCL 点云转换为 ROS2 PointCloud2 消息并发布
    void publishDepthCloud(const pcl::PointCloud<pcl::PointXYZRGB> &colored_cloud,
                           const std_msgs::msg::Header &header);
};
