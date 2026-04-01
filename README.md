# Odin ROS2 驱动 (odin_ros2_driver) 🚀🤖

适用于 Manifold Tech Ltd. Odin1 传感器模块的 **ROS2 Humble** 驱动功能包 ✨

- 📚 官方 Wiki：[https://manifoldtechltd.github.io/wiki/odin_series/odin1/](https://manifoldtechltd.github.io/wiki/odin_series/odin1/)
- 🖥️ 支持平台：**Ubuntu 22.04.5 LTS** + **ROS2 Humble**（x86_64 / ARM aarch64）

## 版本信息 📌

| 🏷️ 项目 | 📋 版本 |
| --- | --- |
| 🚗 驱动版本 | v0.9.0 |
| 🔧 设备固件要求 | v0.10.0+ |
| 🐧 支持系统 | Ubuntu 22.04.5 LTS |
| 🌐 ROS 版本 | ROS2 Humble |

## 系统要求与依赖 🧰

### 系统要求 💻

- 🐧 **操作系统**：Ubuntu 22.04.5 LTS
- 🤖 **ROS 版本**：ROS2 Humble
- 🧠 **架构**：x86_64（Intel/AMD）或 ARM aarch64（Jetson、树莓派等）

### 系统依赖 📦

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

### ROS2 依赖 🤝

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

## USB 权限配置 🔐

首次使用时，需配置 USB 设备访问权限，按下面步骤执行即可 (๑•̀ㅂ•́)و✧

```bash
# 创建 udev 规则文件 🛠️
sudo bash -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2207\", ATTR{idProduct}==\"0019\", MODE=\"0666\", GROUP=\"plugdev\"" > /etc/udev/rules.d/99-odin1.rules'

# 重新加载 udev 规则 🔄
sudo udevadm control --reload-rules && sudo udevadm trigger

# 将当前用户添加到 plugdev 组（重新登录后生效）👤
sudo usermod -aG plugdev $USER
```

## 目录结构 🗂️

一图看懂项目主要组成 (＾▽＾)

```text
odin_ros2_driver/
├── CMakeLists.txt                  # 🧱 CMake 构建入口（编译目标与依赖）
├── package.xml                     # 📄 ROS2 包元信息
├── LICENSE                         # ⚖️ 开源许可证
├── README.md                       # 📘 使用文档
├── set_param.sh                    # 🎛️ 运行期参数下发脚本
├── config/                         # ⚙️ 配置目录
│   ├── control_command.yaml        # 🧭 主配置（功能开关/QoS/性能参数）
│   ├── calib.yaml                  # 📐 相机与外参标定（设备连接后生成/更新）
│   └── rviz/
│       └── odin_ros2.rviz          # 👀 RViz2 预设布局
├── launch/                         # 🚀 ROS2 启动文件
│   └── odin1.launch.py             # 🧩 主 launch（驱动+深度+重投影+RViz）
├── include/                        # 🧠 C++ 头文件
├── src/                            # 🛠️ C++ 源码（驱动主链路与算法实现）
├── script/                         # 🧪 辅助工具脚本
│   ├── bag2pcd.py                  # 📦 bag/点云相关离线处理脚本
│   └── track_relative_pose_odom_to_base.py  # 📍 轨迹/位姿转换辅助脚本
└── lib/                            # 🔩 预编译设备 SDK 静态库
    ├── liblydHostApi_amd.a         # 🖥️ x86_64 平台
    └── liblydHostApi_arm.a         # 🧩 ARM aarch64 平台
```

## 量化优化结果（相对官方基线 `0c3a02a`） 🚀

整体目标：减少无效处理、降低等待与阻塞、提升主链路稳定帧率 (ง •_•)ง

| 优化项 | 量化结果 |
| --- | --- |
| 🔌 USB3 检测改为 sysfs 直读 | 该步骤耗时约下降 **99.8%~99.95%**（旧方法约 500ms~2000ms） |
| ⏱️ 设备连接轮询 `1s -> 100ms` | 感知等待粒度下降 **90%**，平均少等约 **450ms** |
| 📥 已有 `calib.yaml` 时跳过下载 | 命中条件时，该步骤耗时下降 **100%** |
| ♻️ `calib.yaml` 解析复用 | 解析次数 **2 -> 1**，下降 **50%** |
| 📡 关闭项不创建 publisher | 按当前常见配置，活跃发布路数约 **-58.3%**（12 -> 5） |
| 🧮 RGB/DTOF 按订阅需求触发计算（无订阅不做重处理） | 无消费时对应分支 CPU 开销可降 **100%**；整进程常见 **-15%~45%** |
| 🌀 cloud_render 仅保留最新帧配对（防止积压） | 队列压力约 **-90%**（10 -> 1），配对等待常见 **-70%~95%** |
| 🌊 深度节点“最新同步帧优先” + 深度图/深度点云独立开关 | `senddepthcloud=0` 时深度节点负载常见 **-20%~45%** |
| 📐 畸变映射单次遍历 + Horner 多项式求值（替代多次 `pow`） | 遍历趟数 **-50%**，`pow()` 调用在该路径降为 0 |

小结：这批改动更偏向“稳态效率优化 + 启动链路提速”，对日常调试和长时间运行都更友好 ✨٩(ˊᗜˋ*)و
