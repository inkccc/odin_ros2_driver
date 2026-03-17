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

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include "polynomial_camera.hpp"

#include <memory>

// 点云重投影器：将里程计坐标系下的 SLAM 点云投影到相机图像平面生成可视化结果
class CloudReprojector
{
public:
    // 相机内参结构体：包含鱼眼多项式模型的焦距、主点和畸变系数
    struct CameraParams
    {
        int image_width = 1600;
        int image_height = 1296;
        double A11 = 0.0, A12 = 0.0, A22 = 0.0; // 相机内参矩阵元素（焦距/倾斜系数）
        double u0 = 0.0, v0 = 0.0;               // 主点坐标
        double k2 = 0.0, k3 = 0.0, k4 = 0.0, k5 = 0.0, k6 = 0.0, k7 = 0.0; // 多项式畸变系数
    };

    // 外参结构体：相机-激光雷达、激光雷达-IMU 和相机-IMU 的变换矩阵
    struct ExtrinsicParams
    {
        Eigen::Matrix4d Tcl = Eigen::Matrix4d::Identity();  // 相机到激光雷达（camera to lidar）
        Eigen::Matrix4d Til = Eigen::Matrix4d::Identity();  // 激光雷达到 IMU（lidar to imu，固定值）
        Eigen::Matrix4d Tic = Eigen::Matrix4d::Identity();  // 相机到 IMU（由 Tcl 和 Til 计算得出）
    };

    // 里程计位姿：包含四元数姿态和三维位置
    struct OdomPose
    {
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
    };

    // 默认构造函数
    CloudReprojector();
    ~CloudReprojector() = default;

    // 初始化：设置内参和外参，创建多项式相机模型
    bool initialize(const CameraParams& cam_params, const ExtrinsicParams& ext_params);

    // 将里程计坐标系下的彩色点云重投影到相机图像，返回可视化 BGR 图像
    cv::Mat reprojectCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_odom,
                           const OdomPose& odom_pose);

    // 设置/获取点云投影点的绘制半径（像素）
    void setPointRadius(int radius) { point_radius_ = radius; }
    int getPointRadius() const { return point_radius_; }

    // 获取当前相机内参和外参
    const CameraParams& getCameraParams() const { return camera_params_; }
    const ExtrinsicParams& getExtrinsicParams() const { return extrinsic_params_; }

    // 静态工具函数：根据 Tcl 和 Til 计算 Tic = Til * Tcl_inverse
    static Eigen::Matrix4d calculateTic(const Eigen::Matrix4d& Tcl, const Eigen::Matrix4d& Til);

private:
    // 将里程计位姿转换为 4×4 变换矩阵
    Eigen::Matrix4d odomPoseToMatrix(const OdomPose& odom) const;

    // 将相机坐标系下的点云投影到图像平面并绘制
    cv::Mat projectCloudToImage(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in_cam) const;

    CameraParams camera_params_;
    ExtrinsicParams extrinsic_params_;
    std::unique_ptr<mini_vikit::PolynomialCamera> camera_model_;

    int point_radius_ = 4;
    bool initialized_ = false;
};
