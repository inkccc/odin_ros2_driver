# Copyright 2025 Manifold Tech Ltd.
# Licensed under the Apache License, Version 2.0
#
# 使用方式: ros2 launch odin_ros2_driver odin1.launch.py
#
# 本 launch 文件启动 Odin1 传感器驱动所需的全部节点：
#   1. host_sdk_sample              - 主驱动节点，负责设备通信及数据发布
#   2. pcd2depth_ros2_node          - 点云转深度图节点（需在 control_command.yaml 中启用 senddepth: 1）
#   3. cloud_reprojection_ros2_node - 点云重投影节点（需在 control_command.yaml 中启用 sendreprojection: 1）
#   4. rviz2                        - 可视化工具，加载预设 config/rviz/odin_ros2.rviz 配置

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description() -> LaunchDescription:
    # ===== 获取功能包的安装共享目录 =====
    # 用于构建配置文件和 rviz 文件的绝对路径
    package_dir = get_package_share_directory('odin_ros2_driver')

    # ===== 声明 launch 参数：驱动配置文件路径 =====
    # 默认指向 config/control_command.yaml，可在 launch 时通过命令行覆盖：
    #   ros2 launch odin_ros2_driver odin1.launch.py config_file:=/path/to/your/config.yaml
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            package_dir,
            'config',
            'control_command.yaml'
        ),
        description='驱动控制参数配置文件路径（YAML 格式）'
    )

    # ===== 声明 launch 参数：RViz2 配置文件路径 =====
    # 默认使用 config/rviz/odin_ros2.rviz，可在 launch 时通过命令行覆盖：
    #   ros2 launch odin_ros2_driver odin1.launch.py rviz_config:=/path/to/your.rviz
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            package_dir,
            'config',
            'rviz',
            'odin_ros2.rviz'
        ),
        description='RViz2 可视化配置文件路径（.rviz 格式）'
    )

    # ===== 主驱动节点：host_sdk_sample =====
    # 功能：与 Odin1 设备建立 USB3.0 连接，接收并发布以下数据：
    #   - /odin1/cloud_raw        原始 DTOF 点云
    #   - /odin1/cloud_slam       SLAM 点云
    #   - /odin1/cloud_render     彩色渲染点云
    #   - /odin1/image            RGB 图像（BGR8）
    #   - /odin1/image/compressed 压缩 JPEG 图像
    #   - /odin1/imu              IMU 数据
    #   - /odin1/odometry         里程计数据
    #   - tf                      坐标变换（odom → base_link）
    action_host_sdk_sample = Node(
        package='odin_ros2_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        # 如需开启 debug 日志，取消下一行注释：
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    # ===== 深度图生成节点：pcd2depth_ros2_node =====
    # 功能：订阅原始点云和 RGB 图像，融合生成稠密深度图
    # 发布话题：
    #   - /odin1/depth_img_completion       稠密深度图（32FC1 格式）
    #   - /odin1/depth_img_completion_cloud  带颜色的深度点云
    # 注意：需在 control_command.yaml 中设置 senddepth: 1 才会实际发布
    # 注意：此节点运算量较大，可能对主机性能有较高要求
    config_path_pcd2depth = os.path.join(
        package_dir,
        'config',
        'control_command.yaml'
    )
    with open(config_path_pcd2depth, 'r') as f:
        pcd2depth_params = yaml.safe_load(f)
    # [FIX Item 6] calib.yaml 路径改为 source 目录
    # host_sdk_sample 设备连接后将 calib.yaml 下载并保存到 source 目录的 config/；
    # 原来指向 install/share/... 的静态路径不会被 host_sdk_sample 更新，
    # 修复后 pcd2depth_ros2_node 与 host_sdk_sample 读写同一文件，确保使用最新设备标定
    # 此文件位于 launch/ 目录，向上两级即为包根目录（odin_ros2_driver/）
    # 注意：必须使用 os.path.realpath 而非 os.path.abspath：
    #   colcon --symlink-install 时此文件是 install/share/.../launch/ 下的符号链接，
    #   abspath 不解析符号链接，导致路径仍指向 install 目录（无 calib.yaml）；
    #   realpath 解析符号链接，正确指向 src/odin_ros2_driver/
    package_source_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    pcd2depth_calib_path = os.path.join(package_source_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path
    action_pcd2depth_ros2_node = Node(
        package='odin_ros2_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    # ===== 点云重投影节点：cloud_reprojection_ros2_node =====
    # 功能：将 SLAM 点云通过里程计位姿投影到相机图像平面，生成可视化图像
    # 发布话题：
    #   - /odin1/reprojected_image  带点云重投影的相机图像（BGR8）
    # 注意：需在 control_command.yaml 中设置 sendreprojection: 1 才会实际发布
    config_path_cloud_reprojection = os.path.join(
        package_dir,
        'config',
        'control_command.yaml'
    )
    with open(config_path_cloud_reprojection, 'r') as f:
        reprojection_params = yaml.safe_load(f)
    # [FIX Item 6] 同 pcd2depth，使用 source 目录路径保持一致
    # （cloud_reprojection_ros2_node 内部自行通过 __FILE__ 计算路径，此参数作为备用路径记录）
    reprojection_calib_path = os.path.join(package_source_dir, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path
    action_cloud_reprojection_ros2_node = Node(
        package='odin_ros2_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )

    # ===== RViz2 可视化节点 =====
    # 功能：加载预设 rviz 配置，可视化点云、图像、TF 等数据
    # 配置文件位于：config/rviz/odin_ros2.rviz
    action_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )



    return LaunchDescription([
        config_file_arg,
        rviz_config_arg,
        action_host_sdk_sample,
        action_pcd2depth_ros2_node,
        action_cloud_reprojection_ros2_node,
        action_rviz2,
    ])
