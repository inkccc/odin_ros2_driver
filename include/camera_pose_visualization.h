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

#ifdef ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#else
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#endif
#include <Eigen/Dense>
#include <Eigen/Geometry>

// 相机位姿可视化类：生成 RViz2 中显示的相机视锥体、轨迹边和回环边的 MarkerArray 消息
class camera_pose_visualization {
  public:
    std::string m_marker_ns;

    // 构造函数：设置视锥体颜色（RGBA）
    camera_pose_visualization(float r, float g, float b, float a);

    // 设置图像边界线颜色
    void setImageBoundaryColor(float r, float g, float b, float a = 1.0);
    // 设置光心连接线颜色
    void setOpticalCenterConnectorColor(float r, float g, float b, float a = 1.0);
    // 设置视锥体缩放比例
    void setScale(double s);
    // 设置线条宽度
    void setLineWidth(double width);

    // 添加一个相机位姿（视锥体）可视化
    void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
    // 清空所有标记，准备下一帧
    void reset();

    #ifdef ROS2
    using ColorRGBA = std_msgs::msg::ColorRGBA;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Header = std_msgs::msg::Header;
    using Publisher = rclcpp::Publisher<MarkerArray>;
    #else
    using ColorRGBA = std_msgs::ColorRGBA;
    using Marker = visualization_msgs::Marker;
    using MarkerArray = visualization_msgs::MarkerArray;
    using Header = std_msgs::Header;
    using Publisher = ros::Publisher;
    #endif

    // 将所有标记通过 Publisher 发布为 MarkerArray
    void publish_by(Publisher& pub, const Header& header);
    // 添加普通轨迹边（绿色）
    void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
    // 添加回环检测边（紫色）
    void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
  private:
    std::vector<Marker> m_markers;
    ColorRGBA m_image_boundary_color;
    ColorRGBA m_optical_center_connector_color;
    double m_scale;
    double m_line_width;

    static const Eigen::Vector3d imlt;
    static const Eigen::Vector3d imlb;
    static const Eigen::Vector3d imrt;
    static const Eigen::Vector3d imrb;
    static const Eigen::Vector3d oc  ;
    static const Eigen::Vector3d lt0 ;
    static const Eigen::Vector3d lt1 ;
    static const Eigen::Vector3d lt2 ;
};
