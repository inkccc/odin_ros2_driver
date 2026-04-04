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

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include "depth_image_ros2_node.hpp"
#include <rcpputils/filesystem_helper.hpp>
// 检查指定路径的文件是否存在
bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

// 从 calib.yaml 文件加载相机标定参数，并注册到指定 ROS2 节点的参数服务器
// log_intrinsics / log_extrinsics：由 control_command.yaml 中对应 flag 控制是否打印详细参数日志
bool loadCalibParameters(std::shared_ptr<rclcpp::Node> node, const std::string& calib_file_path,
                         bool log_intrinsics, bool log_extrinsics) {
    try {
        RCLCPP_INFO(node->get_logger(), "Loading parameters from calib.yaml: %s", calib_file_path.c_str());

        YAML::Node config = YAML::LoadFile(calib_file_path);

        if (config["cam_num"]) {
            int cam_num = config["cam_num"].as<int>();
            node->declare_parameter("cam_num", cam_num);
        }

        if (config["img_topic_0"]) {
            std::string img_topic = config["img_topic_0"].as<std::string>();
            node->declare_parameter("img_topic_0", img_topic);
        }

        // [LOG] 外参 Tcl_0：受 log_calib_extrinsics flag 控制
        if (config["Tcl_0"]) {
            std::vector<double> tcl_matrix = config["Tcl_0"].as<std::vector<double>>();
            node->declare_parameter("Tcl_0", tcl_matrix);
            if (log_extrinsics) {
                RCLCPP_INFO(node->get_logger(),
                    "[calib] Tcl_0 (camera-to-lidar, %zu elements):", tcl_matrix.size());
                RCLCPP_INFO(node->get_logger(),
                    "  [%.6f  %.6f  %.6f  %.6f]", tcl_matrix[0],  tcl_matrix[1],  tcl_matrix[2],  tcl_matrix[3]);
                RCLCPP_INFO(node->get_logger(),
                    "  [%.6f  %.6f  %.6f  %.6f]", tcl_matrix[4],  tcl_matrix[5],  tcl_matrix[6],  tcl_matrix[7]);
                RCLCPP_INFO(node->get_logger(),
                    "  [%.6f  %.6f  %.6f  %.6f]", tcl_matrix[8],  tcl_matrix[9],  tcl_matrix[10], tcl_matrix[11]);
                RCLCPP_INFO(node->get_logger(),
                    "  [%.6f  %.6f  %.6f  %.6f]", tcl_matrix[12], tcl_matrix[13], tcl_matrix[14], tcl_matrix[15]);
            }
        }

        if (config["cam_0"]) {
            YAML::Node cam_0 = config["cam_0"];

            if (cam_0["cam_model"]) {
                node->declare_parameter("cam_0.cam_model", cam_0["cam_model"].as<std::string>());
            }
            if (cam_0["image_width"]) {
                node->declare_parameter("cam_0.image_width", cam_0["image_width"].as<int>());
            }
            if (cam_0["image_height"]) {
                node->declare_parameter("cam_0.image_height", cam_0["image_height"].as<int>());
            }

            std::vector<std::string> distortion_params = {"k2", "k3", "k4", "k5", "k6", "k7", "p1", "p2"};
            for (const auto& param : distortion_params) {
                if (cam_0[param]) node->declare_parameter("cam_0." + param, cam_0[param].as<double>());
            }

            std::vector<std::string> intrinsic_params = {"A11", "A12", "A22", "u0", "v0"};
            for (const auto& param : intrinsic_params) {
                if (cam_0[param]) node->declare_parameter("cam_0." + param, cam_0[param].as<double>());
            }

            if (cam_0["isFast"])           node->declare_parameter("cam_0.isFast",           cam_0["isFast"].as<int>());
            if (cam_0["numDiff"])          node->declare_parameter("cam_0.numDiff",           cam_0["numDiff"].as<int>());
            if (cam_0["maxIncidentAngle"]) node->declare_parameter("cam_0.maxIncidentAngle",  cam_0["maxIncidentAngle"].as<int>());

            // [LOG] 内参日志：受 log_calib_intrinsics flag 控制
            if (log_intrinsics) {
                RCLCPP_INFO(node->get_logger(), "[calib] Camera intrinsics (pcd2depth_ros2_node):");
                RCLCPP_INFO(node->get_logger(), "  Image size: %dx%d",
                    cam_0["image_width"].as<int>(), cam_0["image_height"].as<int>());
                RCLCPP_INFO(node->get_logger(),
                    "  fx(A11)=%.6f  skew(A12)=%.6f  fy(A22)=%.6f  cx(u0)=%.6f  cy(v0)=%.6f",
                    cam_0["A11"].as<double>(), cam_0["A12"].as<double>(), cam_0["A22"].as<double>(),
                    cam_0["u0"].as<double>(),  cam_0["v0"].as<double>());
                RCLCPP_INFO(node->get_logger(),
                    "  Distortion: k2=%.6f  k3=%.6f  k4=%.6f  k5=%.6f  k6=%.6f  k7=%.6f",
                    cam_0["k2"].as<double>(), cam_0["k3"].as<double>(), cam_0["k4"].as<double>(),
                    cam_0["k5"].as<double>(), cam_0["k6"].as<double>(), cam_0["k7"].as<double>());
            }
        }

        RCLCPP_INFO(node->get_logger(), "Successfully loaded all parameters from calib.yaml");
        return true;

    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(node->get_logger(), "YAML parsing error while loading calib.yaml: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error loading calib parameters: %s", e.what());
        return false;
    }
}
// 通过当前源文件路径向上遍历目录树，定位包含 package.xml 的功能包根目录
std::string get_package_source_directory() {
    // 使用 rcpputils::fs::path 替代 std::filesystem::path
    rcpputils::fs::path current_file(__FILE__);
    
    // 回溯到包根目录
    auto path = current_file.parent_path();
    
    // 使用 rcpputils::fs::exists 替代 std::filesystem::exists
    while (!path.empty() && !rcpputils::fs::exists(path / "package.xml")) {
        path = path.parent_path();
    }
    
    if (path.empty()) {
        throw std::runtime_error("Failed to locate package root directory");
    }
    
    return path.string();
}

// 解析最终生效的 control_command.yaml 路径。
// 优先使用 launch 传入 config_file，未传或路径错误时回退默认配置并打印 WARN。
static std::string resolve_control_config_path(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& default_config_file)
{
    std::string external_config_file = node->declare_parameter<std::string>("config_file", "");

    if (external_config_file.empty()) {
        RCLCPP_WARN(node->get_logger(),
            "[config] launch 参数 `config_file` 未传递，回退到驱动默认配置: %s",
            default_config_file.c_str());
        return default_config_file;
    }

    if (!std::filesystem::exists(external_config_file)) {
        RCLCPP_WARN(node->get_logger(),
            "[config] launch 参数 `config_file` 路径无效: %s，回退到驱动默认配置: %s",
            external_config_file.c_str(), default_config_file.c_str());
        return default_config_file;
    }

    RCLCPP_INFO(node->get_logger(),
        "[config] 使用 launch 传入配置文件: %s", external_config_file.c_str());
    return external_config_file;
}
// 主函数：读取配置确认是否启用深度图，等待 calib.yaml 生成后加载标定参数并启动深度图节点
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    

    auto node = std::make_shared<rclcpp::Node>("pcd2depth_node");
    RCLCPP_INFO(node->get_logger(), "Node created");
    std::string package_path = get_package_source_directory();
    RCLCPP_INFO(node->get_logger(), "Package path: %s", package_path.c_str());
    
    std::string default_config_file = package_path + "/config/control_command.yaml";
    std::string config_file = resolve_control_config_path(node, default_config_file);
    RCLCPP_INFO(node->get_logger(), "Loading config from: %s", config_file.c_str());
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_file);
    } catch (const std::exception& e) {
        if (config_file != default_config_file) {
            RCLCPP_WARN(node->get_logger(),
                "[config] 外部配置解析失败: %s，回退到驱动默认配置: %s，err=%s",
                config_file.c_str(), default_config_file.c_str(), e.what());
            config_file = default_config_file;
            config = YAML::LoadFile(config_file);
        } else {
            throw;
        }
    }
        
    if (!config["register_keys"]) {
            throw std::runtime_error("Missing 'register_keys' section");
        }
        
    if (!config["register_keys"]["senddepth"]) {
            throw std::runtime_error("Missing 'senddepth' parameter");
        }  
    int senddepth = config["register_keys"]["senddepth"].as<int>();
    if(senddepth == 0)
    {
        RCLCPP_INFO(node->get_logger(), "Depth image will not be published (senddepth=0).");
        return 0;
    }

    // [LOG] 读取日志控制 flag（来自 control_command.yaml 的 register_keys 节点）
    auto get_flag = [&](const std::string& key, int default_val) -> int {
        if (config["register_keys"][key]) {
            try { return config["register_keys"][key].as<int>(); } catch (...) {}
        }
        return default_val;
    };
    bool log_calib_intrinsics = get_flag("log_calib_intrinsics", 0) != 0;
    bool log_calib_extrinsics = get_flag("log_calib_extrinsics", 0) != 0;
    bool log_calib_waiting    = get_flag("log_calib_waiting",    1) != 0;
    bool publish_depth_cloud  = get_flag("senddepthcloud",       1) != 0;
    bool depth_postprocess    = get_flag("depth_postprocess",    1) != 0;
    RCLCPP_INFO(node->get_logger(),
        "[LOG] log_calib_intrinsics=%d  log_calib_extrinsics=%d  log_calib_waiting=%d  publish_depth_cloud=%d  depth_postprocess=%d",
        (int)log_calib_intrinsics, (int)log_calib_extrinsics, (int)log_calib_waiting,
        (int)publish_depth_cloud, (int)depth_postprocess);

    std::string calib_file_path = node->declare_parameter<std::string>("calib_file_path", "");

    // [Item 5] 等待 calib.yaml：轮询日志受 log_calib_waiting 控制（默认开启，关闭后仅首条提示保留）
    RCLCPP_INFO(node->get_logger(), "Waiting for calib.yaml at: %s", calib_file_path.c_str());
    while(rclcpp::ok() && !fileExists(calib_file_path))
    {
        if (log_calib_waiting) {
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
                "Still waiting for calib.yaml... (set log_calib_waiting=0 to suppress)");
        }
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        rclcpp::spin_some(node);
    }
    
    if(!rclcpp::ok())
    {
        RCLCPP_INFO(node->get_logger(), "Node shutdown before calib.yaml file was found.");
        return 0;
    }
    
    RCLCPP_INFO(node->get_logger(), "Found calib.yaml file! Loading parameters...");
    
    if (!loadCalibParameters(node, calib_file_path, log_calib_intrinsics, log_calib_extrinsics)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load parameters from calib.yaml file");
        return 1;
    }
    
    rclcpp::NodeOptions depth_node_options;
    
    std::vector<rclcpp::Parameter> params_override;
    
    try {
        params_override.push_back(rclcpp::Parameter("cam_0.image_width", node->get_parameter("cam_0.image_width").as_int()));
        params_override.push_back(rclcpp::Parameter("cam_0.image_height", node->get_parameter("cam_0.image_height").as_int()));
        params_override.push_back(rclcpp::Parameter("cam_0.A11", node->get_parameter("cam_0.A11").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.A12", node->get_parameter("cam_0.A12").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.A22", node->get_parameter("cam_0.A22").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.u0", node->get_parameter("cam_0.u0").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.v0", node->get_parameter("cam_0.v0").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k2", node->get_parameter("cam_0.k2").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k3", node->get_parameter("cam_0.k3").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k4", node->get_parameter("cam_0.k4").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k5", node->get_parameter("cam_0.k5").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k6", node->get_parameter("cam_0.k6").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k7", node->get_parameter("cam_0.k7").as_double()));
        params_override.push_back(rclcpp::Parameter("Tcl_0", node->get_parameter("Tcl_0").as_double_array()));
        params_override.push_back(rclcpp::Parameter("publish_depth_cloud", publish_depth_cloud));
        params_override.push_back(rclcpp::Parameter("depth_postprocess", depth_postprocess));
        
        depth_node_options.parameter_overrides(params_override);
        
        RCLCPP_INFO(node->get_logger(), "Parameters successfully prepared for DepthImageRos2Node");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error preparing parameters for DepthImageRos2Node: %s", e.what());
        return 1;
    }
    try 
    {
        auto depth_node = std::make_shared<DepthImageRos2Node>(depth_node_options);
        
        depth_node->initialize();
        
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(depth_node);
        executor.spin();
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Error creating depth node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
