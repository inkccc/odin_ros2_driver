#!/usr/bin/env python3
"""
ROS2 Bag 点云合并工具 - 将 /odin1/cloud_slam topic 的所有帧合并为单个 PCD 文件。

功能:
    读取 ROS2 bag 文件中的 SLAM 点云数据，将所有帧合并为一张完整的地图点云，
    并保存为 PCD 格式供后续可视化或处理使用。

并行处理:
    使用多线程并行解码点云帧，大幅提升处理速度（CPU 密集型操作）。

进度显示:
    集成 tqdm 进度条，实时显示处理进度和预计剩余时间。

依赖安装:
    pip install numpy tqdm
    # rosbag2_py 和 sensor_msgs_py 已随 ROS2 Humble 安装

用法:
    1. 修改本脚本顶部的 CONFIG 配置区域
    2. 运行: python3 tmp.py
"""

# =============================================================================
# 第一部分：配置区域 (CONFIG) - 用户修改以下参数
# =============================================================================

# -----------------------------------------------------------------------------
# A. 输入/输出路径配置
# -----------------------------------------------------------------------------

# INPUT_BAG_PATH: str
#     bag 文件目录路径，必须包含 metadata.yaml 和 .db3 文件。
#     支持 ~ 展开为用户主目录。
# 示例:
#     "/home/user/ros2_bags/my_bag"
#     "~/workspace/ros2_bags/session_2024_01"
INPUT_BAG_PATH: str = "~/inkc/tws/odinws/files/b2"

# OUTPUT_DIRECTORY: str | None
#     输出 PCD 文件的目录路径。
#     - "." 或 "./"  : 输出到当前工作目录
#     - None         : 输出到 bag 文件所在目录
#     - 具体路径     : 输出到指定目录（如果不存在会自动创建）
# 示例:
#     "."
#     "/home/user/pointclouds"
#     None
OUTPUT_DIRECTORY: str = "."

# OUTPUT_FILENAME: str | None
#     输出文件名（不含 .pcd 扩展名）。
#     - None         : 自动生成，格式为 <bag_name>_merged.pcd
#     - 具体名称     : 使用指定名称
# 示例:
#     "slam_map"
#     "merged_cloud"
#     None
OUTPUT_FILENAME: str | None = None

# -----------------------------------------------------------------------------
# B. 并行处理性能配置
# -----------------------------------------------------------------------------

# NUM_WORKERS: int | None
#     并行处理的线程数，影响点云解码阶段的 CPU 利用率。
#     点云解码是 CPU 密集型操作（numpy 数组转换、RGB 解包等），适合多线程并行。
#     - None         : 自动使用 CPU 核心数 (os.cpu_count())
#     - 1            : 单线程（用于调试）
#     - N            : 使用 N 个线程
# 建议:
#     - 普通 PC (4-8 核): 保持 None 或设置为 4-8
#     - 高性能工作站 (16+ 核): 可设置为 16-32
#     - 如果同时运行其他任务: 设置为 CPU 核心数 - 1
NUM_WORKERS: int | None = None

# SHOW_PROGRESS: bool
#     是否显示 tqdm 进度条。
#     - True  : 显示彩色进度条，包含处理速度、剩余时间、完成百分比
#     - False : 静默模式，只输出日志
SHOW_PROGRESS: bool = True

# -----------------------------------------------------------------------------
# C. ROS2 Bag 读取配置
# -----------------------------------------------------------------------------

# TOPIC_NAME: str
#     要读取的点云 topic 名称。
#     Odin SLAM 默认发布到 /odin1/cloud_slam
# 常见值:
#     "/odin1/cloud_slam"    - Odin 设备的 SLAM 地图
#     "/velodyne_points"     - Velodyne 激光雷达原始点云
#     "/lidar/points"        - 通用激光雷达 topic
TOPIC_NAME: str = "/odin1/cloud_slam"

# BAG_STORAGE_ID: str
#     rosbag2 存储后端类型。
#     ROS2 Humble 默认使用 sqlite3 存储格式（metadata.yaml + .db3 文件）。
# 可选值:
#     "sqlite3"  - SQLite 数据库格式（默认，最常用）
#     "mcap"     - MCAP 格式（ROS2 Galactic+ 支持，更高效）
BAG_STORAGE_ID: str = "sqlite3"

# BAG_SERIALIZATION_FORMAT: str
#     消息序列化格式。
#     ROS2 默认使用 CDR (Common Data Representation) 格式。
# 可选值:
#     "cdr"  - Common Data Representation（默认）
BAG_SERIALIZATION_FORMAT: str = "cdr"

# -----------------------------------------------------------------------------
# D. PCD 文件格式配置
# -----------------------------------------------------------------------------

# PCD_VERSION: str
#     PCD 文件格式版本号。
#     0.7 是 PCL (Point Cloud Library) 使用的标准版本。
PCD_VERSION: str = "0.7"

# PCD_VIEWPOINT: str
#     默认视角参数，描述点云的坐标系和方向。
#     格式: "x y z qw qx qy qz"
#     - x y z   : 视点位置（相机/传感器位置）
#     - qw qx qy qz : 方向四元数（单位四元数表示无旋转）
# 默认值 "0 0 0 1 0 0 0" 表示视点在原点，无旋转。
PCD_VIEWPOINT: str = "0 0 0 1 0 0 0"

# PCD_DATA_FORMAT: str
#     点云数据的存储格式。
#     - "binary" : 二进制格式，文件体积小，读写速度快（推荐）
#     - "ascii"  : ASCII 文本格式，人类可读但体积大、速度慢
# 建议:
#     始终使用 "binary"，除非需要手动查看文件内容。
PCD_DATA_FORMAT: str = "binary"

# PCD_FLOAT_SIZE: int
#     浮点数字段的大小（字节）。
#     float32 = 4 字节，这是 ROS 点云和 PCL 的标准格式。
#     float64 = 8 字节（极少使用）。
PCD_FLOAT_SIZE: int = 4

# -----------------------------------------------------------------------------
# E. 输出文件命名配置
# -----------------------------------------------------------------------------

# OUTPUT_FILE_SUFFIX: str
#     当 OUTPUT_FILENAME 为 None 时，自动生成的文件名后缀。
#     最终文件名格式: <bag_name>{OUTPUT_FILE_SUFFIX}.pcd
# 示例:
#     bag_name = "session_2024_01"
#     suffix = "_merged"
#     结果: "session_2024_01_merged.pcd"
OUTPUT_FILE_SUFFIX: str = "_merged"

# OUTPUT_FILE_EXTENSION: str
#     输出文件的扩展名。
#     标准 PCD 文件使用 ".pcd" 扩展名。
OUTPUT_FILE_EXTENSION: str = ".pcd"

# -----------------------------------------------------------------------------
# F. 日志输出配置
# -----------------------------------------------------------------------------

# 日志前缀用于区分消息级别，便于过滤和脚本解析。
# 格式: [LEVEL] 消息内容
LOG_PREFIX_INFO: str = "[INFO]"
LOG_PREFIX_ERROR: str = "[ERROR]"
LOG_PREFIX_WARNING: str = "[WARNING]"


# =============================================================================
# 第二部分：导入依赖
# =============================================================================

import os
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import NamedTuple

import numpy as np
from tqdm import tqdm

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


# =============================================================================
# 第三部分：数据结构与类型定义
# =============================================================================

class PointCloudFrame(NamedTuple):
    """
    存储单帧点云的 numpy 数组数据。

    作为多线程处理的结果容器，包含坐标和可选的颜色信息。
    """
    xyz: np.ndarray          # 形状 (N, 3) 的 float32 坐标数组
    rgb: np.ndarray | None     # 形状 (N, 3) 的 uint8 颜色数组 [R,G,B]，无颜色则为 None


# =============================================================================
# 第四部分：核心处理函数
# =============================================================================

def process_single_frame(msg: PointCloud2, has_rgb: bool) -> PointCloudFrame:
    """
    处理单帧 PointCloud2 消息，提取为 numpy 数组。

    设计为纯计算函数，无状态、无副作用，线程安全。
    处理流程:
        1. 使用 pc2.read_points 读取点云数据
        2. 转换为结构化 numpy 数组
        3. 提取 xyz 坐标（始终存在）
        4. 如有颜色，解包 RGB 数据

    Args:
        msg: ROS2 PointCloud2 消息对象
        has_rgb: 该消息是否包含 RGB 颜色字段

    Returns:
        PointCloudFrame 命名元组，包含 xyz 和可选的 rgb 数组
    """
    if has_rgb:
        # 读取带颜色的点云: x, y, z, rgb
        gen = pc2.read_points(
            msg,
            field_names=("x", "y", "z", "rgb"),
            skip_nans=True
        )
        points_list = list(gen)

        if not points_list:
            return PointCloudFrame(np.zeros((0, 3), dtype=np.float32), None)

        # 转换为结构化数组 [(x,y,z,rgb), ...]
        points_array = np.array(
            points_list,
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)]
        )

        # 提取 xyz 坐标 (N, 3)
        xyz = np.column_stack([
            points_array['x'],
            points_array['y'],
            points_array['z']
        ])

        # RGB 解包: ROS 将 RGB 打包为 float32，需要按 PCL 标准解包
        # 解包流程: float32 -> view as uint32 -> 位运算提取 r,g,b
        rgb_raw = points_array['rgb']
        rgb_int = rgb_raw.view(np.uint32)

        r = ((rgb_int >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_int >> 8) & 0xFF).astype(np.uint8)
        b = (rgb_int & 0xFF).astype(np.uint8)
        rgb = np.column_stack([r, g, b])

        return PointCloudFrame(xyz, rgb)

    else:
        # 读取纯坐标点云: x, y, z
        gen = pc2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )
        points_list = list(gen)

        if not points_list:
            return PointCloudFrame(np.zeros((0, 3), dtype=np.float32), None)

        points_array = np.array(
            points_list,
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)]
        )

        xyz = np.column_stack([
            points_array['x'],
            points_array['y'],
            points_array['z']
        ])

        return PointCloudFrame(xyz, None)


def read_all_cloud_slam(bag_path: str, topic: str = TOPIC_NAME) -> list[PointCloud2] | None:
    """
    从 ROS2 bag 文件中顺序读取指定 topic 的所有消息。

    注意: rosbag2 是顺序 I/O，无法并行化读取。所有消息读入内存，
    适合帧数不太大的场景（数百到数千帧）。

    Args:
        bag_path: bag 目录路径
        topic: 目标 topic 名称

    Returns:
        PointCloud2 消息列表，如果 topic 不存在返回 None
    """
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id=BAG_STORAGE_ID
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=BAG_SERIALIZATION_FORMAT,
        output_serialization_format=BAG_SERIALIZATION_FORMAT,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # 检查 topic 存在性
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    if topic not in type_map:
        available = [t.name for t in topic_types]
        print(f"{LOG_PREFIX_ERROR} Topic '{topic}' 不存在于 bag 中")
        print(f"        可用 topics: {available}")
        return None

    # 顺序读取所有目标消息
    all_messages: list[PointCloud2] = []
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t == topic:
            msg = deserialize_message(data, PointCloud2)
            all_messages.append(msg)

    print(f"{LOG_PREFIX_INFO} 共读取 {len(all_messages)} 帧 {topic}")
    return all_messages


def merge_pointcloud2_parallel(
    msgs: list[PointCloud2],
    num_workers: int | None = None,
    show_progress: bool = True
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """
    并行处理多帧点云，合并为单个数组。

    处理流程:
        1. 检测第一帧的格式（RGB 或纯 XYZ）
        2. 使用 ThreadPoolExecutor 并行处理所有帧
        3. 使用 tqdm 显示实时进度
        4. 合并所有结果，保持帧顺序

    Args:
        msgs: PointCloud2 消息列表
        num_workers: 并行线程数，None 表示自动
        show_progress: 是否显示进度条

    Returns:
        (xyz, r, g, b) 元组，r/g/b 在无颜色时为 None
    """
    if not msgs:
        print(f"{LOG_PREFIX_ERROR} 没有点云数据可合并")
        return np.array([]), None, None, None

    # 假设所有帧格式一致，检测第一帧
    field_names = [f.name for f in msgs[0].fields]
    has_rgb = "rgb" in field_names

    total_frames = len(msgs)
    print(f"{LOG_PREFIX_INFO} 开始处理 {total_frames} 帧点云 (线程数: {num_workers or '自动'})")

    # 预分配结果列表，按索引存放以保持顺序
    results: list[PointCloudFrame] = [None] * total_frames

    # 并行处理
    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        # 提交所有任务，记录索引
        future_to_index = {
            executor.submit(process_single_frame, msg, has_rgb): i
            for i, msg in enumerate(msgs)
        }

        # 进度条迭代器
        iterator = tqdm(
            as_completed(future_to_index),
            total=total_frames,
            desc="处理点云帧",
            unit="帧",
            ncols=80,
            disable=not show_progress
        )

        # 收集结果
        for future in iterator:
            idx = future_to_index[future]
            try:
                results[idx] = future.result()
            except Exception as e:
                print(f"{LOG_PREFIX_ERROR} 处理第 {idx} 帧时出错: {e}")
                results[idx] = PointCloudFrame(np.zeros((0, 3), dtype=np.float32), None)

    # 过滤空帧
    valid_results = [r for r in results if len(r.xyz) > 0]

    if not valid_results:
        print(f"{LOG_PREFIX_ERROR} 所有帧点云为空")
        return np.array([]), None, None, None

    print(f"{LOG_PREFIX_INFO} 有效帧: {len(valid_results)}/{total_frames}")
    print(f"{LOG_PREFIX_INFO} 合并点云数据...")

    # 向量化合并（比逐帧 append 快得多）
    all_xyz = [r.xyz for r in valid_results]
    merged_xyz = np.vstack(all_xyz)

    if has_rgb:
        all_rgb = [r.rgb for r in valid_results if r.rgb is not None]
        if all_rgb:
            merged_rgb = np.vstack(all_rgb)
            return merged_xyz, merged_rgb[:, 0], merged_rgb[:, 1], merged_rgb[:, 2]

    return merged_xyz, None, None, None


def save_merged_pcd(
    output_path: str,
    xyz: np.ndarray,
    r: np.ndarray | None = None,
    g: np.ndarray | None = None,
    b: np.ndarray | None = None
) -> None:
    """
    将合并后的点云保存为 PCD 文件。

    支持两种格式:
        - XYZRGB: 带颜色，PCL 标准格式
        - XYZ: 纯坐标

    Args:
        output_path: 完整输出路径
        xyz: (N, 3) 坐标数组
        r, g, b: 可选的颜色通道数组
    """
    n = len(xyz)

    if r is not None and g is not None and b is not None:
        # RGB 打包为 PCL 标准格式
        rgb_packed = (
            r.astype(np.uint32) << 16 |
            g.astype(np.uint32) << 8 |
            b.astype(np.uint32)
        )
        rgb_float = rgb_packed.view(np.float32)

        with open(output_path, "wb") as f:
            header = (
                f"# .PCD v{PCD_VERSION} - Point Cloud Data\n"
                f"VERSION {PCD_VERSION}\n"
                f"FIELDS x y z rgb\n"
                f"SIZE {PCD_FLOAT_SIZE} {PCD_FLOAT_SIZE} {PCD_FLOAT_SIZE} {PCD_FLOAT_SIZE}\n"
                f"TYPE F F F F\n"
                f"COUNT 1 1 1 1\n"
                f"WIDTH {n}\n"
                f"HEIGHT 1\n"
                f"VIEWPOINT {PCD_VIEWPOINT}\n"
                f"POINTS {n}\n"
                f"DATA {PCD_DATA_FORMAT}\n"
            )
            f.write(header.encode("ascii"))

            data = np.column_stack([xyz, rgb_float]).astype(np.float32)
            f.write(data.tobytes())

        print(f"{LOG_PREFIX_INFO} 已保存（XYZRGB，{PCD_DATA_FORMAT}）: {output_path}  ({n:,} 点)")
    else:
        with open(output_path, "wb") as f:
            header = (
                f"# .PCD v{PCD_VERSION} - Point Cloud Data\n"
                f"VERSION {PCD_VERSION}\n"
                f"FIELDS x y z\n"
                f"SIZE {PCD_FLOAT_SIZE} {PCD_FLOAT_SIZE} {PCD_FLOAT_SIZE}\n"
                f"TYPE F F F\n"
                f"COUNT 1 1 1\n"
                f"WIDTH {n}\n"
                f"HEIGHT 1\n"
                f"VIEWPOINT {PCD_VIEWPOINT}\n"
                f"POINTS {n}\n"
                f"DATA {PCD_DATA_FORMAT}\n"
            )
            f.write(header.encode("ascii"))
            f.write(xyz.astype(np.float32).tobytes())

        print(f"{LOG_PREFIX_INFO} 已保存（XYZ，{PCD_DATA_FORMAT}）: {output_path}  ({n:,} 点)")


# =============================================================================
# 第五部分：脚本入口
# =============================================================================

def main() -> None:
    """主执行函数：配置检查 -> 读取 -> 处理 -> 保存。"""

    # ---- 1. 路径解析与验证 ----
    bag_path = os.path.expanduser(INPUT_BAG_PATH)
    if not os.path.isdir(bag_path):
        print(f"{LOG_PREFIX_ERROR} bag 目录不存在: {bag_path}")
        sys.exit(1)

    # 确定输出目录
    if OUTPUT_DIRECTORY is None:
        output_dir = bag_path
    else:
        output_dir = os.path.expanduser(OUTPUT_DIRECTORY)

    os.makedirs(output_dir, exist_ok=True)

    # 确定输出文件名
    if OUTPUT_FILENAME is not None:
        filename = f"{OUTPUT_FILENAME}{OUTPUT_FILE_EXTENSION}"
    else:
        bag_name = os.path.basename(bag_path.rstrip("/"))
        filename = f"{bag_name}{OUTPUT_FILE_SUFFIX}{OUTPUT_FILE_EXTENSION}"

    output_path = os.path.join(output_dir, filename)

    print(f"{LOG_PREFIX_INFO} 输入 bag: {bag_path}")
    print(f"{LOG_PREFIX_INFO} 输出 PCD: {output_path}")

    # ---- 2. 读取 bag 数据（顺序 I/O） ----
    msgs = read_all_cloud_slam(bag_path, TOPIC_NAME)
    if msgs is None:
        sys.exit(1)

    # ---- 3. 并行处理点云 ----
    xyz, r, g, b = merge_pointcloud2_parallel(
        msgs,
        num_workers=NUM_WORKERS,
        show_progress=SHOW_PROGRESS
    )

    if len(xyz) == 0:
        print(f"{LOG_PREFIX_ERROR} 点云数据为空，无法保存")
        sys.exit(1)

    # ---- 4. 保存结果 ----
    save_merged_pcd(output_path, xyz, r, g, b)

    print(f"{LOG_PREFIX_INFO} 转换完成!")


if __name__ == "__main__":
    main()
