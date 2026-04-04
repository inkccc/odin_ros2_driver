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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ===== 获取功能包的安装共享目录 =====
    # 用于构建配置文件和 rviz 文件的绝对路径
    package_path_odin_ros2_driver = get_package_share_directory('odin_ros2_driver')
    # ===== 内置固定配置路径（按需求去掉 launch 参数）=====
    config_path_odin_ros2_driver_control = '/home/inkc/inkc/tws/odinws/src/odin_ros2_driver/config/control_command.yaml'
    config_path_rviz2_odin_ros2_driver = '/home/inkc/inkc/tws/odinws/src/odin_ros2_driver/config/rviz/odin_ros2.rviz'

    # [FIX Item 6] calib.yaml 路径改为 source 目录
    # host_sdk_sample 设备连接后将 calib.yaml 下载并保存到 source 目录的 config/；
    # 原来指向 install/share/... 的静态路径不会被 host_sdk_sample 更新。
    package_source_dir_odin_ros2_driver = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    config_path_odin_ros2_driver_calib_pcd2depth = os.path.join(package_source_dir_odin_ros2_driver, 'config', 'calib.yaml')
    config_path_odin_ros2_driver_calib_reprojection = os.path.join(package_source_dir_odin_ros2_driver, 'config', 'calib.yaml')

    # ===== 主驱动节点：host_sdk_sample =====
    # 通过 config_file 参数向源码传入外部配置路径；
    # 源码侧负责路径校验与 fallback（未传/路径错误 -> WARN + 默认 YAML）。
    action_odin_ros2_driver_host_sdk_sample = Node(
        package='odin_ros2_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[{
            'config_file': config_path_odin_ros2_driver_control
        }]
    )

    # ===== 深度图生成节点：pcd2depth_ros2_node =====
    # 这里直接传 config_file + calib_file_path，不在 launch 中解析 control_command.yaml，
    # 由源码统一读取配置，确保与 host_sdk_sample 的配置来源一致。
    action_odin_ros2_driver_pcd2depth_ros2_node = Node(
        package='odin_ros2_driver',
        executable='pcd2depth_ros2_node',
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[{
            'config_file': config_path_odin_ros2_driver_control,
            'calib_file_path': config_path_odin_ros2_driver_calib_pcd2depth
        }]
    )

    # ===== 点云重投影节点：cloud_reprojection_ros2_node =====
    # 与 pcd2depth 保持一致：配置路径统一由 config_file 驱动，避免跨节点配置源分叉。
    action_odin_ros2_driver_cloud_reprojection_ros2_node = Node(
        package='odin_ros2_driver',
        executable='cloud_reprojection_ros2_node',
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[{
            'config_file': config_path_odin_ros2_driver_control,
            'calib_file_path': config_path_odin_ros2_driver_calib_reprojection
        }]
    )

    # ===== RViz2 可视化节点 =====
    # 功能：加载预设 rviz 配置，可视化点云、图像、TF 等数据
    # 配置文件位于：config/rviz/odin_ros2.rviz
    action_rviz2_odin_ros2_driver = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_path_rviz2_odin_ros2_driver]
    )

    return LaunchDescription([
        action_odin_ros2_driver_host_sdk_sample,
        action_odin_ros2_driver_pcd2depth_ros2_node,
        action_odin_ros2_driver_cloud_reprojection_ros2_node,
        action_rviz2_odin_ros2_driver,
    ])
