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
#ifndef YAML_PARSER_H
#define YAML_PARSER_H

#include <cstdio>
#include <string>
#include <map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "lidar_api.h"

namespace odin_ros2_driver {

// 参数值数据类型枚举，用于区分整型、浮点数组和整型数组
enum class DataType {
    INT_TYPE,
    FLOAT_ARRAY_TYPE,
    INT_ARRAY_TYPE,
};

// 通用参数值容器，以字节数组形式存储不同类型的参数值
struct ParameterValue {
    DataType type;
    std::vector<uint8_t> data;

    ParameterValue() : type(DataType::INT_TYPE) {}

    template<typename T>
    void setData(const T& value) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
        data.assign(ptr, ptr + sizeof(T));
    }

    template<typename T>
    void setArray(const std::vector<T>& arr) {
        const uint8_t* ptr = reinterpret_cast<const uint8_t*>(arr.data());
        data.assign(ptr, ptr + arr.size() * sizeof(T));
    }

    size_t getSize() const {
        return data.size();
    }

    const void* getData() const {
        return data.empty() ? nullptr : data.data();
    }
};

// YAML 配置解析器：读取 control_command.yaml，提取 register_keys 下的各类参数
// 支持整型参数、字符串参数（路径等）和以 custom_ 为前缀的设备自定义参数
class YamlParser {
public:
    // 构造函数，传入配置文件路径
    YamlParser(const std::string& config_file);

    // 加载并解析配置文件
    bool loadConfig();
    // 获取整型参数映射表
    const std::map<std::string, int>& getRegisterKeys() const;
    // 获取字符串参数映射表
    const std::map<std::string, std::string>& getRegisterKeysStrVal() const;
    // 获取自定义参数映射表
    const std::map<std::string, ParameterValue>& getCustomParameters() const;
    // 打印所有已加载参数（用于调试）
    void printConfig() const;
    // 将所有自定义参数通过 lidar API 发送到设备
    bool applyCustomParameters(device_handle device);
    // 获取指定名称的自定义整型参数，不存在则返回默认值
    int getCustomParameterInt(const std::string& param_name, int default_value) const;

    // 获取 custom_map_mode 参数（地图工作模式）
    int getCustomMapMode(int default_value) const {
        auto it = custom_parameters_.find("map_mode");
        if (it != custom_parameters_.end() && it->second.type == DataType::INT_TYPE) {
            printf("custom_map_mode = %d\n", *(int*)it->second.getData());
            return *(int*)it->second.getData();
        } else {
            return default_value;
        }
    };

private:
    std::string config_file_;
    std::map<std::string, int> register_keys_;
    std::map<std::string, std::string> register_keys_str_val_;
    std::map<std::string, ParameterValue> custom_parameters_;

    // 允许以字符串形式解析的非 custom_ 前缀参数白名单
    // 路径类参数和帧名称参数需要在此声明，否则解析时会尝试转为 int 并报错
    std::unordered_set<std::string> allowed_key_w_str_val = {
        "relocalization_map_abs_path",   // 重定位地图文件绝对路径
        "mapping_result_dest_dir",       // SLAM 地图保存目录
        "mapping_result_file_name",      // SLAM 地图保存文件名
        "odom_child_frame",              // odom TF 子帧名（底盘模式下 odom 的 child_frame_id）
        "sensor_frame_id",               // 传感器帧名（点云 frame_id 及传感器 TF child_frame_id，默认 "odin1_base_link"）
    };

    // 仅主机端使用、不发送到设备 SDK 的 custom_ 参数名白名单
    // applyCustomParameters() 会跳过此集合中的参数，避免向设备发送未知参数导致错误
    std::unordered_set<std::string> host_only_custom_params = {
        "base_to_sensor_tf",             // 传感器到底盘的安装外参（仅主机端 odom TF 计算使用）
    };
};

}

#endif