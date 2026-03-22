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

// ROS2 Humble 头文件
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "cloud_reprojector.hpp"

#include <string>
#include <memory>

// 点云重投影 ROS2 节点：订阅 SLAM 点云和里程计，将点云投影到相机图像并发布可视化结果
class CloudReprojectionRosNode : public rclcpp::Node
{
public:
    // 构造函数：加载参数并初始化订阅器、发布器和重投影处理器
    CloudReprojectionRosNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
    using Odometry = nav_msgs::msg::Odometry;
    using Image = sensor_msgs::msg::Image;

    // 订阅的话题名称
    std::string cloud_slam_topic_;
    std::string odometry_topic_;
    std::string reprojected_image_topic_;

    // 时间同步的点云和里程计订阅器
    message_filters::Subscriber<PointCloud2> cloud_sub_;
    message_filters::Subscriber<Odometry> odom_sub_;

    // 精确时间同步策略
    typedef message_filters::sync_policies::ExactTime<PointCloud2, Odometry> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    std::shared_ptr<Sync> sync_;

    // 重投影图像发布器
    image_transport::Publisher reprojected_image_pub_;

    // 点云重投影核心处理器
    std::unique_ptr<CloudReprojector> reprojector_;

    // 从 ROS2 参数服务器和 calib.yaml 加载配置
    void loadParameters();

    // 时间同步回调：执行重投影并发布图像
    void syncCallback(const PointCloud2::ConstSharedPtr& cloud_msg,
                      const Odometry::ConstSharedPtr& odom_msg);
};
