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
#ifndef RGBCLOUD_H
#define RGBCLOUD_H

#include <vector>
#include <string>
#include <Eigen/Dense>
#include "lidar_api.h"
#include <yaml-cpp/yaml.h>

namespace GlobalCameraParams {
    extern float g_fx;
    extern float g_fy;
    extern float g_cx;
    extern float g_cy;
    extern float g_skew;
    extern float g_k2;
    extern float g_k3;
    extern float g_k4;
    extern float g_k5;
    extern float g_k6;
    extern float g_k7;
    extern Eigen::Matrix4f g_T_camera_lidar;
}

// [LOG] 日志控制全局变量：由 host_sdk_sample::main() 从 control_command.yaml 读取后赋值
// 在 rawCloudRender::init() 调用之前必须已赋值；默认均为 false（静默模式）
extern bool g_log_calib_intrinsics;  // 控制内参日志（焦距、主点、畸变系数）
extern bool g_log_calib_extrinsics;  // 控制外参日志（Tcl 相机-激光雷达变换矩阵）

// 原始点云彩色渲染器：将 DTOF 点云通过相机外参投影到 RGB 图像，生成彩色点云
class rawCloudRender {
public:

    // 从 calib.yaml 加载相机内参（畸变系数、焦距等）和外参（相机-激光雷达变换矩阵）
    bool init(const std::string& yamlFilePath);

    // [OPT-4] 从已解析的 YAML::Node 初始化，避免重复解析同一文件
    bool init(const YAML::Node& config);

    // 将设备 NV12 格式图像缓冲区转换为 RGB 浮点二维数组
    void nv12buffer_2_rgb(buffer_List_t &image, std::vector<std::vector<float>>& rgb_image);

    // 将点云通过相机投影模型映射到图像，生成彩色点云（XYZRGB 展平格式）
    // 优化版：直接接受扁平化的 BGR float 缓冲区，避免每帧重建二维 vector。
    void render(const float* rgb_image_flat, int img_width, int img_height,
                capture_Image_List_t* pcdStream, int pcdIdx, std::vector<float>& rgbCloud_flat);

    // 打印当前相机标定参数（用于调试）
    void print_camera_calib();

    // 获取图像宽度（像素）
    int getImageWidth() const { return image_width_; }
    // 获取图像高度（像素）
    int getImageHeight() const { return image_height_; }
    
   

private:
    std::string model_type_;
    std::string camera_name_;
    int image_width_;
    int image_height_;
    int frame_size_;
    bool opencv_available_;
    
    // 4x4 transformation matrix (T_camera_lidar)
    Eigen::Matrix4f T_camera_lidar_;

    float k2_;
    float k3_;
    float k4_;
    float k5_;
    float k6_;
    float k7_;
    float p1_;
    float p2_;
    float A11_fx_;
    float A12_skew_;
    float A22_fy_;
    float u0_cx_;
    float v0_cy_;
    bool isFast_;
    int numDiff_;
    float maxIncidentAngle_; 
    

};

#endif  // RGBCLOUD_H
