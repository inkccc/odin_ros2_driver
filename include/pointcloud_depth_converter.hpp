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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>


// 点云转深度图转换器：将稀疏 DTOF 点云投影为稠密深度图，并与 RGB 图像融合生成彩色点云
class PointCloudToDepthConverter
{
public:
    // 相机参数结构体：包含内参、畸变系数、缩放比例和相机-激光雷达外参矩阵
    struct CameraParams
    {
        int image_width;
        int image_height;
        double A11, A12, A22;           // 相机内参：焦距和倾斜系数
        double u0, v0;                  // 相机主点坐标
        double k2, k3, k4, k5, k6, k7; // 多项式畸变系数（鱼眼模型）
        double scale;                   // 深度图缩放比例（降采样因子）
        int point_sampling_rate;        // 彩色点云生成时的像素采样间隔
        Eigen::Matrix4d Tcl;            // 相机到激光雷达的变换矩阵
    };

    // 处理结果结构体：包含深度图、彩色点云及状态信息
    struct ProcessResult
    {
        cv::Mat depth_image;
        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
        bool success;
        std::string error_message;
    };

    // 构造函数：初始化相机参数，预计算投影矩阵和畸变映射表
    explicit PointCloudToDepthConverter(const CameraParams &params);

    // 主处理接口：将点云和 RGB 图像融合，输出深度图和彩色点云
    ProcessResult processCloudAndImage(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                       const cv::Mat &image);

    // 深度图最近邻插值放大（保留有效深度值）
    cv::Mat customResize(const cv::Mat& src, const cv::Size& size);
    // 获取当前相机参数
    const CameraParams &getCameraParams() const { return params_; }

    // 动态更新相机参数并重新初始化内部状态
    void updateCameraParams(const CameraParams &params);

private:
    CameraParams params_;

    Eigen::Matrix3d K_;
    Eigen::Matrix3d Kl_;
    Eigen::Matrix4d K_4x4_;
    Eigen::Matrix4d Kcl_;
    Eigen::Matrix4d Tlc_; // 预计算的 Tcl 逆矩阵（激光雷达到相机），避免每帧重复求逆

    cv::Mat map_x_, map_y_;
    cv::Mat inv_map_x_, inv_map_y_;

    int scaled_width_, scaled_height_;


    void initializeInternalParams();

 
    void createDistortionMaps();

    cv::Mat projectCloudToDepth(const pcl::PointCloud<pcl::PointXYZ> &cloud_in_cam);


    cv::Mat postProcessDepthImage(const cv::Mat &depth_img);


    pcl::PointCloud<pcl::PointXYZRGB> generateColoredCloud(const cv::Mat &depth_img,
                                                           const cv::Mat &color_img);


    std::pair<bool, std::string> validateInputs(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                const cv::Mat &image);
};