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

#include "pointcloud_depth_converter.hpp"
#include <cmath>
#include <iostream>

// 构造函数：根据相机参数初始化深度图转换器，预计算相机矩阵和畸变映射表
PointCloudToDepthConverter::PointCloudToDepthConverter(const CameraParams &params)
    : params_(params)
{
    initializeInternalParams();
    createDistortionMaps();
}

// 根据相机参数计算缩放尺寸、相机内参矩阵 K 及相机-激光雷达联合投影矩阵 Kcl
void PointCloudToDepthConverter::initializeInternalParams()
{
    scaled_width_ = static_cast<int>(params_.image_width / params_.scale);
    scaled_height_ = static_cast<int>(params_.image_height / params_.scale);

    K_ = Eigen::Matrix3d::Identity();
    K_(0, 0) = params_.A11;
    K_(0, 1) = params_.A12;
    K_(0, 2) = params_.u0;
    K_(1, 1) = params_.A22;
    K_(1, 2) = params_.v0;

    Kl_ = Eigen::Matrix3d::Identity();
    Kl_(0, 0) = params_.A11 / params_.scale;
    Kl_(0, 1) = 0.0;
    Kl_(0, 2) = params_.u0 / params_.scale;
    Kl_(1, 1) = params_.A22 / params_.scale;
    Kl_(1, 2) = params_.v0 / params_.scale;

    K_4x4_ = Eigen::Matrix4d::Identity();
    K_4x4_.block<3, 3>(0, 0) = Kl_;

    Kcl_ = K_4x4_ * params_.Tcl;
    Tlc_ = params_.Tcl.inverse();
}

// 预计算鱼眼相机的正向和逆向畸变映射表，用于图像去畸变和点云投影
// 优化：合并正向/逆向两次遍历为单次遍历，使用 Horner 方法替代 pow() 计算多项式
void PointCloudToDepthConverter::createDistortionMaps()
{
    map_x_ = cv::Mat::zeros(params_.image_height, params_.image_width, CV_32FC1);
    map_y_ = cv::Mat::zeros(params_.image_height, params_.image_width, CV_32FC1);
    inv_map_x_ = cv::Mat::zeros(params_.image_height, params_.image_width, CV_32FC1);
    inv_map_y_ = cv::Mat::zeros(params_.image_height, params_.image_width, CV_32FC1);

    for (int u = 0; u < params_.image_width; ++u)
    {
        for (int v = 0; v < params_.image_height; ++v)
        {
            double y = (v - params_.v0) / params_.A22;
            double x = (u - params_.u0 - params_.A12 * y) / params_.A11;

            double r = sqrt(x * x + y * y);
            double theta = atan(r);

            // Horner 方法计算 theta_d = theta + k2*theta^2 + ... + k7*theta^7
            double theta_d = theta * (1.0 + theta * (params_.k2 + theta * (params_.k3 + theta * (params_.k4
                           + theta * (params_.k5 + theta * (params_.k6 + theta * params_.k7))))));

            // 正向映射: scaling = r / theta_d
            double fwd_x = x * (r / theta_d);
            double fwd_y = y * (r / theta_d);
            map_x_.at<float>(v, u) = static_cast<float>(fwd_x * params_.A11 + params_.A12 * fwd_y + params_.u0);
            map_y_.at<float>(v, u) = static_cast<float>(fwd_y * params_.A22 + params_.v0);

            // 逆向映射: scaling = theta_d / r
            double inv_x = x * (theta_d / r);
            double inv_y = y * (theta_d / r);
            inv_map_x_.at<float>(v, u) = static_cast<float>(inv_x * params_.A11 + params_.A12 * inv_y + params_.u0);
            inv_map_y_.at<float>(v, u) = static_cast<float>(inv_y * params_.A22 + params_.v0);
        }
    }
}

// 主处理函数：将输入点云投影到相机坐标系生成深度图，并与 RGB 图像融合输出彩色点云
PointCloudToDepthConverter::ProcessResult PointCloudToDepthConverter::processCloudAndImage(
    const pcl::PointCloud<pcl::PointXYZ> &cloud,
    const cv::Mat &image,
    bool enable_post_processing,
    bool generate_colored_cloud)
{
    ProcessResult result;
    result.success = false;

    auto validation_result = validateInputs(cloud, image);
    if (!validation_result.first)
    {
        result.error_message = validation_result.second;
        return result;
    }

    try
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_in_cam;
        pcl::transformPointCloud(cloud, cloud_in_cam, Kcl_);

        cv::Mat depth_img = projectCloudToDepth(cloud_in_cam);

        // 为兼容历史下游，哪怕关闭深度后处理，也仍然先恢复到原始图像分辨率，
        // 这样发布的话题格式和尺寸保持稳定，只是去掉 Sobel/阈值去噪阶段。
        cv::Mat processed_depth = enable_post_processing
            ? postProcessDepthImage(depth_img)
            : upscaleDepthImage(depth_img);

        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
        if (generate_colored_cloud) {
            colored_cloud = generateColoredCloud(processed_depth, image);
        }

        result.depth_image = processed_depth;
        result.colored_cloud = colored_cloud;
        result.success = true;
    }
    catch (const std::exception &e)
    {
        result.error_message = std::string("Processing error: ") + e.what();
    }

    return result;
}

// 将相机坐标系下的点云投影到缩放后的深度图像（最近邻填充 3×3 邻域）
cv::Mat PointCloudToDepthConverter::projectCloudToDepth(const pcl::PointCloud<pcl::PointXYZ> &cloud_in_cam)
{
    cv::Mat depth_img = cv::Mat::zeros(scaled_height_, scaled_width_, CV_32FC1);

    for (const auto &camera_point : cloud_in_cam)
    {
        if (camera_point.z <= 0)
            continue; 

        int u = static_cast<int>(std::round(camera_point.x / camera_point.z));
        int v = static_cast<int>(std::round(camera_point.y / camera_point.z));

        if (u >= 0 && u < scaled_width_ && v >= 0 && v < scaled_height_)
        {
            depth_img.at<float>(v, u) = static_cast<float>(camera_point.z);

            for (int du = -1; du <= 1; ++du)
            {
                for (int dv = -1; dv <= 1; ++dv)
                {
                    int nu = u + du;
                    int nv = v + dv;
                    if (nu >= 0 && nu < scaled_width_ && nv >= 0 && nv < scaled_height_)
                    {
                        if (depth_img.at<float>(nv, nu) == 0.0f)
                        {
                            depth_img.at<float>(nv, nu) = static_cast<float>(camera_point.z);
                        }
                    }
                }
            }
        }
    }

    return depth_img;
}

cv::Mat PointCloudToDepthConverter::upscaleDepthImage(const cv::Mat &depth_img) {
    if (depth_img.empty()) {
        std::cerr << "ERROR: Input depth image is empty!" << std::endl;
        return cv::Mat();
    }
    
    if (depth_img.data == nullptr) {
        std::cerr << "ERROR: Input depth image has null data pointer!" << std::endl;
        return cv::Mat();
    }
    
    if (depth_img.rows <= 0 || depth_img.cols <= 0) {
        std::cerr << "ERROR: Invalid input dimensions: " 
                  << depth_img.rows << "x" << depth_img.cols << std::endl;
        return cv::Mat();
    }
    
    if (params_.image_width <= 0 || params_.image_height <= 0) {
        std::cerr << "ERROR: Invalid target size: "
                  << params_.image_width << "x" << params_.image_height << std::endl;
        return cv::Mat();
    }

    cv::Mat depth_img_upsampled;
    try {
        depth_img_upsampled = customResize(depth_img, cv::Size(params_.image_width, params_.image_height));
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Custom resize failed: " << e.what() << std::endl;
        return cv::Mat();
    }
    
    if (depth_img_upsampled.empty()) {
        std::cerr << "ERROR: Resized image is empty!" << std::endl;
        return cv::Mat();
    }
    
    if (depth_img_upsampled.rows != params_.image_height || depth_img_upsampled.cols != params_.image_width) {
        std::cerr << "ERROR: Resized image has wrong dimensions: " 
                  << depth_img_upsampled.cols << "x" << depth_img_upsampled.rows
                  << " (expected " << params_.image_width << "x" << params_.image_height << ")" << std::endl;
        return cv::Mat();
    }

    return depth_img_upsampled;
}

// 对深度图进行后处理：先恢复到原始分辨率，再用 Sobel 梯度阈值去除边缘噪声
cv::Mat PointCloudToDepthConverter::postProcessDepthImage(const cv::Mat &depth_img) {
    cv::Mat depth_img_upsampled = upscaleDepthImage(depth_img);
    if (depth_img_upsampled.empty()) {
        return cv::Mat();
    }

    cv::Mat grad_x, grad_y, grad_magnitude;
    try {
        cv::Sobel(depth_img_upsampled, grad_x, CV_32F, 1, 0, 3);
        cv::Sobel(depth_img_upsampled, grad_y, CV_32F, 0, 1, 3);
        cv::magnitude(grad_x, grad_y, grad_magnitude);
    } catch (const cv::Exception& e) {
        std::cerr << "ERROR: Sobel/magnitude failed: " << e.what() << std::endl;
        return cv::Mat();
    }
    
    if (grad_magnitude.type() != CV_32F) {
        std::cerr << "ERROR: grad_magnitude has wrong type: " 
                  << grad_magnitude.type() << " (expected CV_32F)" << std::endl;
        return cv::Mat();
    }
    

    cv::Mat threshold_mask;
    try {
        cv::threshold(grad_magnitude, threshold_mask, 0.75, 1, cv::THRESH_BINARY);
        threshold_mask.convertTo(threshold_mask, CV_8U);
        

        depth_img_upsampled.setTo(0, threshold_mask);
    } catch (const cv::Exception& e) {
        std::cerr << "ERROR: Threshold mask failed: " << e.what() << std::endl;
        return cv::Mat();
    }

    return depth_img_upsampled;
}

// 对单通道浮点深度图进行最近邻插值放大，保留有效深度值（不引入插值噪声）
cv::Mat PointCloudToDepthConverter::customResize(const cv::Mat& src, const cv::Size& size) {
    if (src.empty()) {
        throw std::runtime_error("Source image is empty");
    }
    
    if (size.width <= 0 || size.height <= 0) {
        throw std::runtime_error("Invalid target size");
    }
    

    cv::Mat dst(size.height, size.width, src.type());

    float scale_x = src.cols / static_cast<float>(size.width);
    float scale_y = src.rows / static_cast<float>(size.height);

    if (src.channels() != 1 || src.type() != CV_32F) {
        throw std::runtime_error("Unsupported image type - expected single channel float");
    }

    for (int y = 0; y < dst.rows; y++) {
        float* dst_row = dst.ptr<float>(y);
        int src_y = std::min(static_cast<int>(y * scale_y), src.rows - 1);
        const float* src_row = src.ptr<float>(src_y);

        for (int x = 0; x < dst.cols; x++) {
            int src_x = std::min(static_cast<int>(x * scale_x), src.cols - 1);
            dst_row[x] = src_row[src_x];
        }
    }
    
    return dst;
}
// 根据深度图和 RGB 图像反投影生成带颜色的 PCL 点云（激光雷达坐标系）
pcl::PointCloud<pcl::PointXYZRGB> PointCloudToDepthConverter::generateColoredCloud(
    const cv::Mat &depth_img, const cv::Mat &color_img)
{
    cv::Mat color_undistorted;
    cv::remap(color_img, color_undistorted, inv_map_x_, inv_map_y_, cv::INTER_LINEAR);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_colored;
    cloud_colored.points.reserve(
        static_cast<size_t>(depth_img.rows / params_.point_sampling_rate)
        * (depth_img.cols / params_.point_sampling_rate));

    for (int v = 0; v < depth_img.rows; v += params_.point_sampling_rate)
    {
        for (int u = 0; u < depth_img.cols; u += params_.point_sampling_rate)
        {
            float depth = depth_img.at<float>(v, u);
            if (depth > 0.1f && depth < 100.0f) 
            {
                double y_cam = (v - params_.v0) * depth / params_.A22;
                double x_cam = ((u - params_.u0) * depth  - params_.A12 * y_cam)/ params_.A11;
                
                double z_cam = depth;

                Eigen::Vector4d point_cam(x_cam, y_cam, z_cam, 1.0);

                Eigen::Vector4d point_lidar = Tlc_ * point_cam;

                pcl::PointXYZRGB point;
                point.x = static_cast<float>(point_lidar[0]);
                point.y = static_cast<float>(point_lidar[1]);
                point.z = static_cast<float>(point_lidar[2]);

                if (u < color_undistorted.cols && v < color_undistorted.rows)
                {
                    cv::Vec3b color = color_undistorted.at<cv::Vec3b>(v, u);
                    point.b = color[0]; 
                    point.g = color[1];
                    point.r = color[2];
                }
                else
                {
                    point.r = point.g = point.b = 255;
                }

                cloud_colored.points.push_back(point);
            }
        }
    }

    cloud_colored.width = cloud_colored.points.size();
    cloud_colored.height = 1;
    cloud_colored.is_dense = false;

    return cloud_colored;
}

// 校验输入点云和图像的有效性，以及相机内参是否已正确初始化
std::pair<bool, std::string> PointCloudToDepthConverter::validateInputs(
    const pcl::PointCloud<pcl::PointXYZ> &cloud, const cv::Mat &image)
{
    if (cloud.empty())
    {
        return {false, "Empty point cloud"};
    }

    if (image.empty())
    {
        return {false, "Empty image"};
    }

    if (params_.A11 < 1e-6 || params_.A22 < 1e-6)
    {
        return {false, "Invalid camera intrinsics"};
    }

    return {true, ""};
}

// 动态更新相机参数并重新初始化内部参数和畸变映射表
void PointCloudToDepthConverter::updateCameraParams(const CameraParams &params)
{
    params_ = params;
    initializeInternalParams();
    createDistortionMaps();
}
