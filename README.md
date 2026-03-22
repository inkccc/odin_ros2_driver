# Odin ROS2 驱动 (odin_ros2_driver)

适用于 Manifold Tech Ltd. Odin1 传感器模块的 **ROS2 Humble** 驱动功能包。

- 官方 Wiki：[https://manifoldtechltd.github.io/wiki/odin_series/odin1/](https://manifoldtechltd.github.io/wiki/odin_series/odin1/)
- 支持平台：**Ubuntu 22.04.5 LTS** + **ROS2 Humble**（x86_64 / ARM aarch64）

---

## 重要声明

本驱动功能包提供 SLAM 点云应用的核心功能，面向特定使用场景。本软件仅供从事二次开发的技术专业人员使用。终端用户在实际部署环境中必须根据应用场景进行针对性优化和定制开发，以满足实际运营需求。

---

## 1. 版本信息


| 项目     | 版本                 |
| ------ | ------------------ |
| 驱动版本   | v0.9.0             |
| 设备固件要求 | v0.10.0+           |
| 支持系统   | Ubuntu 22.04.5 LTS |
| ROS 版本 | ROS2 Humble        |


---

## 2. 环境准备

### 2.1 系统要求

- **操作系统**：Ubuntu 22.04.5 LTS（Jammy Jellyfish）
- **ROS 版本**：ROS2 Humble Hawksbill
- **架构**：x86_64（Intel/AMD）或 ARM aarch64（Jetson、树莓派等）

### 2.2 安装 ROS2 Humble

若尚未安装 ROS2 Humble，请参考官方文档：
[https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

快速安装（推荐）：

```bash
# 设置软件源
sudo apt install software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list

# 安装 ROS2 Humble 桌面版
sudo apt update && sudo apt install ros-humble-desktop

# 添加环境变量到 ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.3 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
    libopencv-dev \
    libyaml-cpp-dev \
    libssl-dev \
    libeigen3-dev \
    libusb-1.0-0-dev \
    libpcl-dev \
    pkg-config
```

OpenCV 版本要求 >= 4.5.0（推荐 4.5.5 或 4.8.0，请确保系统中只安装了一个版本）。

### 2.4 安装 ROS2 相关依赖

```bash
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-pcl-conversions \
    ros-humble-message-filters \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-visualization-msgs \
    ros-humble-rviz2
```

---

## 3. 设备连接

### 3.1 USB 权限配置

首次使用时，需配置 USB 设备访问权限：

```bash
# 创建 udev 规则文件
sudo bash -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2207\", ATTR{idProduct}==\"0019\", MODE=\"0666\", GROUP=\"plugdev\"" > /etc/udev/rules.d/99-odin1.rules'

# 重新加载 udev 规则
sudo udevadm control --reload-rules && sudo udevadm trigger

# 将当前用户添加到 plugdev 组
sudo usermod -aG plugdev $USER
# 注意：需要重新登录才能生效
```

### 3.2 USB 3.0 要求

- SLAM 模式下**必须**使用 USB 3.0 接口，以确保地图文件传输的可靠性
- 若使用 USB 2.0，可在 `config/control_command.yaml` 中设置 `strict_usb3.0_check: 0` 跳过检查（不推荐）
- 设备 USB ID：`2207:0019`

---

## 4. 编译

### 4.1 创建工作空间并克隆功能包

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <本仓库地址> odin_ros_driver
```

### 4.2 使用 colcon 编译

```bash
cd ~/ros2_ws
# 加载 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译（将自动检测 x86_64 或 ARM 架构）
colcon build --packages-select odin_ros_driver

# 加载工作空间
source install/setup.bash
```

ARM 平台（Jetson、树莓派等）编译命令相同，CMake 会自动检测架构并链接对应的 `liblydHostApi_arm.a` 库。

---

## 5. 启动驱动

### 5.1 基本启动

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch odin_ros_driver odin1.launch.py
```

启动后将自动运行以下节点：

- `host_sdk_sample`：主驱动节点，与 Odin1 设备建立 USB 连接
- `pcd2depth_ros2_node`：深度图生成节点（需在配置文件中启用）
- `cloud_reprojection_ros2_node`：点云重投影节点（需在配置文件中启用）
- `rviz2`：可视化界面

### 5.2 自定义参数启动

```bash
# 使用自定义配置文件
ros2 launch odin_ros_driver odin1.launch.py config_file:=/path/to/your/config.yaml

# 使用自定义 RViz 配置
ros2 launch odin_ros_driver odin1.launch.py rviz_config:=/path/to/your.rviz
```

### 5.3 配置文件

主要配置文件位于 `config/control_command.yaml`，各参数含义详见文件内注释。

主要配置项一览：


| 参数                    | 默认值 | 说明                               |
| --------------------- | --- | -------------------------------- |
| `strict_usb3.0_check` | 0   | USB 3.0 强制检查（推荐生产环境开启）           |
| `use_host_ros_time`   | 2   | 时间戳同步模式（0=设备时间，1=主机接收时间，2=PTP同步） |
| `sendrgb`             | 1   | 发布 BGR8 解码图像                     |
| `senddtof`            | 1   | 发布原始 DTOF 点云                     |
| `sendcloudslam`       | 1   | 发布 SLAM 点云                       |
| `sendcloudrender`     | 1   | 发布彩色渲染点云                         |
| `senddepth`           | 0   | 发布稠密深度图（计算量大，默认关闭）               |
| `sendreprojection`    | 0   | 发布点云重投影图（默认关闭）                   |
| `custom_map_mode`     | 0   | 工作模式（0=里程计，1=SLAM建图，2=重定位）       |


---

## 6. 发布的 ROS2 话题

设备连接后，驱动将发布以下话题：


| 话题名称                          | 消息类型                          | 说明                              |
| ----------------------------- | ----------------------------- | ------------------------------- |
| `/odin1/cloud_raw`            | `sensor_msgs/PointCloud2`     | 原始 DTOF 点云（XYZI + 置信度 + 时间偏移）   |
| `/odin1/cloud_slam`           | `sensor_msgs/PointCloud2`     | SLAM 全局点云地图（XYZRGB）             |
| `/odin1/cloud_render`         | `sensor_msgs/PointCloud2`     | 与 RGB 融合的彩色点云                   |
| `/odin1/image`                | `sensor_msgs/Image`           | 解码 RGB 图像（BGR8）                 |
| `/odin1/image/compressed`     | `sensor_msgs/CompressedImage` | 原始 JPEG 压缩图像                    |
| `/odin1/image_undistort`      | `sensor_msgs/Image`           | 去畸变 RGB 图像（需启用）                 |
| `/odin1/imu`                  | `sensor_msgs/Imu`             | IMU 数据（加速度 + 角速度）               |
| `/odin1/odometry`             | `nav_msgs/Odometry`           | 里程计位姿（位置 + 姿态 + 速度）             |
| `/odin1/odometry_high`        | `nav_msgs/Odometry`           | 高频里程计                           |
| `/odin1/path`                 | `nav_msgs/Path`               | 历史轨迹路径（需启用 showpath）            |
| `/odin1/depth_img_completion` | `sensor_msgs/Image`           | 稠密深度图 32FC1（需启用 senddepth）      |
| `/odin1/reprojected_image`    | `sensor_msgs/Image`           | 点云重投影可视化图（需启用 sendreprojection） |
| `/tf`                         | `tf2_msgs/TFMessage`          | 坐标变换（odom → base_link）          |


### 原始点云格式说明

`/odin1/cloud_raw` 使用自定义点结构：

```
字段: x, y, z (float32), intensity (uint8), confidence (uint16), offset_time (float32)
```

- `confidence` 范围：0 ~ 1300，推荐过滤阈值：30 ~ 35
- `offset_time`：相对于帧基准时间的偏移（秒）

---

## 7. 工作模式说明

### 模式 0：里程计模式（Odometry）

```yaml
custom_map_mode: 0
```

- 仅输出实时位姿估计，不建图
- 计算量最小，适合对实时性要求高的场景

### 模式 1：SLAM 建图模式

```yaml
custom_map_mode: 1
```

- 完整 SLAM 系统，支持回环检测
- 自动缓存地图数据

**保存地图：**

```bash
# 运行期间执行保存命令
./set_param.sh save_map 1
```

地图默认保存至：`{工作空间}/src/odin_ros_driver/map/{启动时间}/`

也可通过配置文件指定：

```yaml
mapping_result_dest_dir: "/home/user/maps/"
mapping_result_file_name: "my_room.bin"
```

### 模式 2：重定位模式（Relocalization）

```yaml
custom_map_mode: 2
relocalization_map_abs_path: "/home/user/maps/my_room.bin"
```

- 加载已有地图进行定位，无需重新建图
- **启动要求**：初始位置需在建图起点 **1 米以内、±10° 角度范围**内
- 重定位失败时自动退回 SLAM 模式
- 成功后发布 `map → odom` 坐标变换

---

## 8. 深度图功能说明

启用稠密深度图补全：

```yaml
senddepth: 1
```

**工作原理**：

1. 订阅 `/odin1/cloud_raw`（稀疏点云）和 `/odin1/image`（RGB 图像）
2. 将点云投影到相机坐标系生成稀疏深度图
3. 上采样至原始分辨率（1600×1296），并用 Sobel 梯度过滤边缘噪声
4. 发布 `32FC1` 格式深度图（单位：米）

**注意**：此功能在主机端进行计算，运算量较大，可能影响整体性能。

---

## 9. 数据录制功能说明

启用 OLX 格式数据录制：

```yaml
recorddata: 1
```

录制的数据包括 RGB 图像、里程计和 SLAM 点云，以专有 OLX 格式保存，可用 MindCloud(TM) 软件进行后处理。

**保存路径**：`{工作空间}/src/odin_ros_driver/recorddata/{录制开始时间}/`

> **注意**：后处理时请完整复制整个文件夹，不要只复制部分文件。

---

## 10. 目录结构

```
odin_ros_driver/
├── CMakeLists.txt           # CMake 构建配置（仅支持 ROS2 Humble）
├── package.xml              # ROS2 功能包描述文件
├── config/
│   └── control_command.yaml # 驱动控制参数配置文件
├── launch/
│   └── odin1.launch.py      # ROS2 启动文件
├── rviz/
│   └── odin_ros2.rviz       # RViz2 可视化配置文件
├── src/                     # C++ 源文件
│   ├── host_sdk_sample.cpp       # 主驱动节点
│   ├── yaml_parser.cpp           # YAML 配置解析器
│   ├── rawCloudRender.cpp        # 原始点云彩色渲染
│   ├── camera_pose_visualization.cpp # 相机位姿可视化
│   ├── depth_image_ros2_node.cpp # 深度图 ROS2 节点
│   ├── pcd2depth_ros2.cpp        # 深度图节点入口
│   ├── pointcloud_depth_converter.cpp # 点云转深度图核心算法
│   ├── cloud_reprojection_ros.cpp    # 点云重投影 ROS2 节点
│   └── cloud_reprojector.cpp         # 点云重投影核心算法
├── include/                 # C++ 头文件
├── lib/
│   ├── liblydHostApi_amd.a  # x86_64 预编译 SDK 库
│   └── liblydHostApi_arm.a  # ARM 预编译 SDK 库
└── set_param.sh             # 运行时参数设置脚本（如保存地图）
```

---

## 11. 常见问题

### 问题 1：设备无法连接

- 检查 USB 线是否为 USB 3.0 规格（蓝色接口）
- 检查 udev 规则是否正确配置：`lsusb | grep 2207`
- 重新登录以使 plugdev 组权限生效

### 问题 2：编译报错 "Could not find lydHostApi library"

- 确认 `lib/` 目录下存在对应架构的库文件（`liblydHostApi_amd.a` 或 `liblydHostApi_arm.a`）
- x86_64 系统使用 amd 库，ARM 系统使用 arm 库

### 问题 3：点云在 RViz2 中不显示

- 确认 `send_odom_baselink_tf: 1`（默认已开启）
- 在 RViz2 中将 Fixed Frame 设置为 `odom`

### 问题 4：SLAM 模式下 USB 传输不稳定

- 必须使用 USB 3.0 接口，避免使用 USB HUB
- 可设置 `strict_usb3.0_check: 1` 强制检查

### 问题 5：时间戳不同步

- 推荐使用 `use_host_ros_time: 2`（PTP 类同步，默认值）
- 若需使用纯设备时间，设置为 `use_host_ros_time: 0`

---

## 12. 技术支持

- **邮件支持**：[support@manifoldtech.cn](mailto:support@manifoldtech.cn)
- **官方 Wiki**：[https://manifoldtechltd.github.io/wiki/Odin1/Cover.html](https://manifoldtechltd.github.io/wiki/Odin1/Cover.html)
- **许可证**：Apache License 2.0
- **版权**：Copyright 2025 Manifold Tech Ltd.

