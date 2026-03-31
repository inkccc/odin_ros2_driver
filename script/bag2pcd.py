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
    2. 运行: python3 bag2pcd.py
"""

# =============================================================================
# 第一部分：配置区域 (CONFIG) - 用户修改以下参数
# =============================================================================

# -----------------------------------------------------------------------------
# A. 输入/输出路径配置
# -----------------------------------------------------------------------------

INPUT_BAG_PATH: str = "~/inkc/tws/odinws/files/b2"
OUTPUT_DIRECTORY: str = "."
OUTPUT_FILENAME: str | None = None

# -----------------------------------------------------------------------------
# B. 并行处理性能配置
# -----------------------------------------------------------------------------

NUM_WORKERS: int | None = None
SHOW_PROGRESS: bool = True

# -----------------------------------------------------------------------------
# C. ROS2 Bag 读取配置
# -----------------------------------------------------------------------------

TOPIC_NAME: str = "/odin1/cloud_slam"
BAG_STORAGE_ID: str = "sqlite3"
BAG_SERIALIZATION_FORMAT: str = "cdr"

# -----------------------------------------------------------------------------
# D. PCD 文件格式配置
# -----------------------------------------------------------------------------

PCD_VERSION: str = "0.7"
PCD_VIEWPOINT: str = "0 0 0 1 0 0 0"
PCD_DATA_FORMAT: str = "binary"
PCD_FLOAT_SIZE: int = 4

# -----------------------------------------------------------------------------
# E. 输出文件命名配置
# -----------------------------------------------------------------------------

OUTPUT_FILE_SUFFIX: str = "_merged"
OUTPUT_FILE_EXTENSION: str = ".pcd"

# -----------------------------------------------------------------------------
# F. 日志输出配置
# -----------------------------------------------------------------------------

LOG_PREFIX_INFO: str = "[INFO]"
LOG_PREFIX_ERROR: str = "[ERROR]"
LOG_PREFIX_WARNING: str = "[WARNING]"

# -----------------------------------------------------------------------------
# G. 点云后处理配置（降采样 + 离群点过滤）
# -----------------------------------------------------------------------------

# --- G1. 体素滤波（Voxel Grid Downsampling）---

VOXEL_ENABLE: bool = True
VOXEL_SIZE: float = 0.02

# --- G2. 离群点过滤 ---

OUTLIER_METHOD: str = "voxel_density"

# --- G2c. Voxel Density 参数（推荐，速度极快）---

VOXEL_DENSITY_SIZE: float = 0.1
VOXEL_DENSITY_MIN_POINTS: int = 3

# --- G2a. Statistical Outlier Removal 参数 ---

SOR_K: int = 20
SOR_STD_RATIO: float = 2.0
SOR_BATCH_SIZE: int = 5_000

# --- G2b. Radius Outlier Removal 参数 ---

ROR_RADIUS: float = 0.5
ROR_MIN_NEIGHBORS: int = 5
ROR_BATCH_SIZE: int = 200


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
    """处理单帧 PointCloud2 消息，提取为 numpy 数组。"""
    if has_rgb:
        gen = pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True)
        points_list = list(gen)
        if not points_list:
            return PointCloudFrame(np.zeros((0, 3), dtype=np.float32), None)
        
        points_array = np.array(
            points_list,
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)]
        )
        xyz = np.column_stack([points_array['x'], points_array['y'], points_array['z']])
        
        rgb_raw = points_array['rgb']
        rgb_int = rgb_raw.view(np.uint32)
        r = ((rgb_int >> 16) & 0xFF).astype(np.uint8)
        g = ((rgb_int >> 8) & 0xFF).astype(np.uint8)
        b = (rgb_int & 0xFF).astype(np.uint8)
        rgb = np.column_stack([r, g, b])
        return PointCloudFrame(xyz, rgb)
    else:
        gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points_list = list(gen)
        if not points_list:
            return PointCloudFrame(np.zeros((0, 3), dtype=np.float32), None)
        
        points_array = np.array(
            points_list,
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)]
        )
        xyz = np.column_stack([points_array['x'], points_array['y'], points_array['z']])
        return PointCloudFrame(xyz, None)


def read_all_cloud_slam(bag_path: str, topic: str = TOPIC_NAME) -> list[PointCloud2] | None:
    """从 ROS2 bag 文件中顺序读取指定 topic 的所有消息。"""
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id=BAG_STORAGE_ID)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=BAG_SERIALIZATION_FORMAT,
        output_serialization_format=BAG_SERIALIZATION_FORMAT,
    )
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}
    
    if topic not in type_map:
        available = [t.name for t in topic_types]
        print(f"{LOG_PREFIX_ERROR} Topic '{topic}' 不存在于 bag 中")
        print(f"        可用 topics: {available}")
        return None
    
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
    """并行处理多帧点云，合并为单个数组。"""
    if not msgs:
        print(f"{LOG_PREFIX_ERROR} 没有点云数据可合并")
        return np.array([]), None, None, None
    
    field_names = [f.name for f in msgs[0].fields]
    has_rgb = "rgb" in field_names
    total_frames = len(msgs)
    print(f"{LOG_PREFIX_INFO} 开始处理 {total_frames} 帧点云 (线程数: {num_workers or '自动'})")
    
    results: list[PointCloudFrame] = [None] * total_frames
    
    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        future_to_index = {
            executor.submit(process_single_frame, msg, has_rgb): i
            for i, msg in enumerate(msgs)
        }
        
        iterator = tqdm(
            as_completed(future_to_index),
            total=total_frames,
            desc="处理点云帧",
            unit="帧",
            ncols=80,
            disable=not show_progress
        )
        
        for future in iterator:
            idx = future_to_index[future]
            try:
                results[idx] = future.result()
            except Exception as e:
                print(f"{LOG_PREFIX_ERROR} 处理第 {idx} 帧时出错: {e}")
                results[idx] = PointCloudFrame(np.zeros((0, 3), dtype=np.float32), None)
    
    valid_results = [r for r in results if len(r.xyz) > 0]
    
    if not valid_results:
        print(f"{LOG_PREFIX_ERROR} 所有帧点云为空")
        return np.array([]), None, None, None
    
    print(f"{LOG_PREFIX_INFO} 有效帧: {len(valid_results)}/{total_frames}")
    print(f"{LOG_PREFIX_INFO} 合并点云数据...")
    
    all_xyz = [r.xyz for r in valid_results]
    merged_xyz = np.vstack(all_xyz)
    
    if has_rgb:
        all_rgb = [r.rgb for r in valid_results if r.rgb is not None]
        if all_rgb:
            merged_rgb = np.vstack(all_rgb)
            return merged_xyz, merged_rgb[:, 0], merged_rgb[:, 1], merged_rgb[:, 2]
    
    return merged_xyz, None, None, None


def voxel_grid_filter(
    xyz: np.ndarray,
    r: np.ndarray | None,
    g: np.ndarray | None,
    b: np.ndarray | None,
    voxel_size: float,
    show_progress: bool = True
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """体素滤波降采样：将空间划分为均匀体素，每格保留质心。"""
    pbar = tqdm(total=6, desc="体素降采样", unit="步", ncols=80, disable=not show_progress)

    pbar.set_postfix_str("计算索引")
    min_bound = xyz.min(axis=0)
    indices = np.floor((xyz - min_bound) / voxel_size).astype(np.int64)
    max_idx = indices.max(axis=0) + 1
    pbar.update(1)

    grid_volume = int(max_idx[0]) * int(max_idx[1]) * int(max_idx[2])
    if grid_volume > 2 ** 62:
        pbar.close()
        print(f"{LOG_PREFIX_WARNING} 体素网格过大 ({grid_volume:.2e})，请增大 VOXEL_SIZE")
        return xyz, r, g, b

    pbar.set_postfix_str("一维编码")
    encoded = (
        indices[:, 0] * (max_idx[1] * max_idx[2])
        + indices[:, 1] * max_idx[2]
        + indices[:, 2]
    )
    pbar.update(1)

    pbar.set_postfix_str("排序")
    sort_order = np.argsort(encoded)
    sorted_encoded = encoded[sort_order]
    sorted_xyz = xyz[sort_order]
    pbar.update(1)

    pbar.set_postfix_str("去重统计")
    _, first_occ, counts = np.unique(sorted_encoded, return_index=True, return_counts=True)
    pbar.update(1)

    pbar.set_postfix_str("计算质心")
    sum_xyz = np.add.reduceat(sorted_xyz, first_occ, axis=0)
    centroid_xyz = (sum_xyz / counts[:, np.newaxis]).astype(np.float32)
    pbar.update(1)

    pbar.set_postfix_str("处理颜色")
    if r is not None and g is not None and b is not None:
        rgb_stack = np.stack([r, g, b], axis=1).astype(np.float32)
        sorted_rgb = rgb_stack[sort_order]
        sum_rgb = np.add.reduceat(sorted_rgb, first_occ, axis=0)
        centroid_rgb = np.clip(sum_rgb / counts[:, np.newaxis], 0, 255).astype(np.uint8)
        pbar.update(1)
        pbar.close()
        return centroid_xyz, centroid_rgb[:, 0], centroid_rgb[:, 1], centroid_rgb[:, 2]

    pbar.update(1)
    pbar.close()
    return centroid_xyz, None, None, None


def voxel_density_filter(
    xyz: np.ndarray,
    r: np.ndarray | None,
    g: np.ndarray | None,
    b: np.ndarray | None,
    voxel_size: float,
    min_points: int,
    show_progress: bool = True
) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """基于体素密度的快速离群点过滤。"""
    pbar = tqdm(total=4, desc="密度过滤", unit="步", ncols=80, disable=not show_progress)

    pbar.set_postfix_str("计算索引")
    min_bound = xyz.min(axis=0)
    indices = np.floor((xyz - min_bound) / voxel_size).astype(np.int64)
    max_idx = indices.max(axis=0) + 1
    pbar.update(1)

    pbar.set_postfix_str("一维编码")
    encoded = (
        indices[:, 0] * (max_idx[1] * max_idx[2])
        + indices[:, 1] * max_idx[2]
        + indices[:, 2]
    )
    pbar.update(1)

    pbar.set_postfix_str("统计密度")
    _, inverse, counts = np.unique(encoded, return_inverse=True, return_counts=True)
    point_counts = counts[inverse]
    pbar.update(1)

    pbar.set_postfix_str("过滤点云")
    keep_mask = point_counts >= min_points
    xyz_filtered = xyz[keep_mask]
    pbar.update(1)
    pbar.close()

    if r is not None and g is not None and b is not None:
        return xyz_filtered, r[keep_mask], g[keep_mask], b[keep_mask]

    return xyz_filtered, None, None, None


def statistical_outlier_removal(
    xyz: np.ndarray,
    k: int,
    std_ratio: float,
    batch_size: int,
    show_progress: bool = True
) -> np.ndarray:
    """Statistical Outlier Removal：基于邻域距离统计过滤离群点。"""
    n = len(xyz)
    overlap = k * 10

    sort_key = xyz[:, 0] + xyz[:, 1] + xyz[:, 2]
    order = np.argsort(sort_key)
    xyz_sorted = xyz[order]

    mean_dists = np.zeros(n, dtype=np.float32)

    num_batches = (n + batch_size - 1) // batch_size
    batch_iter = range(0, n, batch_size)
    if show_progress:
        batch_iter = tqdm(batch_iter, total=num_batches, desc="SOR 过滤", unit="批", ncols=80)

    for start in batch_iter:
        end = min(start + batch_size, n)
        ext_start = max(0, start - overlap)
        ext_end = min(n, end + overlap)

        query = xyz_sorted[start:end]
        context = xyz_sorted[ext_start:ext_end]

        sq_q = (query ** 2).sum(axis=1)
        sq_c = (context ** 2).sum(axis=1)
        dot = query @ context.T
        sq_dists = sq_q[:, None] + sq_c[None, :] - 2.0 * dot
        np.clip(sq_dists, 0, None, out=sq_dists)

        actual_k = min(k + 1, sq_dists.shape[1])
        knn_sq = np.partition(sq_dists, actual_k - 1, axis=1)[:, 1:actual_k]
        mean_dists[start:end] = np.sqrt(knn_sq).mean(axis=1)

    global_mean = mean_dists.mean()
    global_std = mean_dists.std()
    threshold = global_mean + std_ratio * global_std

    keep_sorted = mean_dists <= threshold

    keep_mask = np.zeros(n, dtype=bool)
    keep_mask[order] = keep_sorted
    return keep_mask


def radius_outlier_removal(
    xyz: np.ndarray,
    radius: float,
    min_neighbors: int,
    batch_size: int,
    show_progress: bool = True
) -> np.ndarray:
    """Radius Outlier Removal：半径内邻居数不足的点视为离群点。"""
    n = len(xyz)
    neighbor_counts = np.zeros(n, dtype=np.int32)
    sq_radius = radius ** 2
    sq_all = (xyz ** 2).sum(axis=1)

    num_batches = (n + batch_size - 1) // batch_size
    batch_iter = range(0, n, batch_size)
    if show_progress:
        batch_iter = tqdm(batch_iter, total=num_batches, desc="ROR 过滤", unit="批", ncols=80)

    for start in batch_iter:
        end = min(start + batch_size, n)
        batch = xyz[start:end]

        sq_b = (batch ** 2).sum(axis=1)
        dot = batch @ xyz.T
        sq_dists = sq_b[:, None] + sq_all[None, :] - 2.0 * dot
        np.clip(sq_dists, 0, None, out=sq_dists)

        within = (sq_dists <= sq_radius).sum(axis=1) - 1
        neighbor_counts[start:end] = within

    return neighbor_counts >= min_neighbors


def save_merged_pcd(
    output_path: str,
    xyz: np.ndarray,
    r: np.ndarray | None = None,
    g: np.ndarray | None = None,
    b: np.ndarray | None = None
) -> None:
    """将合并后的点云保存为 PCD 文件。"""
    n = len(xyz)

    if r is not None and g is not None and b is not None:
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

    bag_path = os.path.expanduser(INPUT_BAG_PATH)
    if not os.path.isdir(bag_path):
        print(f"{LOG_PREFIX_ERROR} bag 目录不存在: {bag_path}")
        sys.exit(1)

    if OUTPUT_DIRECTORY is None:
        output_dir = bag_path
    else:
        output_dir = os.path.expanduser(OUTPUT_DIRECTORY)

    os.makedirs(output_dir, exist_ok=True)

    if OUTPUT_FILENAME is not None:
        filename = f"{OUTPUT_FILENAME}{OUTPUT_FILE_EXTENSION}"
    else:
        bag_name = os.path.basename(bag_path.rstrip("/"))
        filename = f"{bag_name}{OUTPUT_FILE_SUFFIX}{OUTPUT_FILE_EXTENSION}"

    output_path = os.path.join(output_dir, filename)

    print(f"{LOG_PREFIX_INFO} 输入 bag: {bag_path}")
    print(f"{LOG_PREFIX_INFO} 输出 PCD: {output_path}")

    msgs = read_all_cloud_slam(bag_path, TOPIC_NAME)
    if msgs is None:
        sys.exit(1)

    xyz, r, g, b = merge_pointcloud2_parallel(
        msgs,
        num_workers=NUM_WORKERS,
        show_progress=SHOW_PROGRESS
    )

    if len(xyz) == 0:
        print(f"{LOG_PREFIX_ERROR} 点云数据为空，无法保存")
        sys.exit(1)

    print(f"{LOG_PREFIX_INFO} 合并后点云: {len(xyz):,} 点")

    if VOXEL_ENABLE:
        xyz, r, g, b = voxel_grid_filter(xyz, r, g, b, VOXEL_SIZE, SHOW_PROGRESS)
        print(f"{LOG_PREFIX_INFO} 降采样后: {len(xyz):,} 点")

    if OUTLIER_METHOD == "voxel_density":
        before = len(xyz)
        xyz, r, g, b = voxel_density_filter(xyz, r, g, b, VOXEL_DENSITY_SIZE, VOXEL_DENSITY_MIN_POINTS, SHOW_PROGRESS)
        print(f"{LOG_PREFIX_INFO} 密度过滤后: {len(xyz):,} 点 (移除 {before - len(xyz):,} 点)")

    elif OUTLIER_METHOD == "statistical":
        keep = statistical_outlier_removal(xyz, SOR_K, SOR_STD_RATIO, SOR_BATCH_SIZE, SHOW_PROGRESS)
        removed = int((~keep).sum())
        xyz = xyz[keep]
        if r is not None:
            r, g, b = r[keep], g[keep], b[keep]
        print(f"{LOG_PREFIX_INFO} SOR 后: {len(xyz):,} 点 (移除 {removed:,} 点)")

    elif OUTLIER_METHOD == "radius":
        keep = radius_outlier_removal(xyz, ROR_RADIUS, ROR_MIN_NEIGHBORS, ROR_BATCH_SIZE, SHOW_PROGRESS)
        removed = int((~keep).sum())
        xyz = xyz[keep]
        if r is not None:
            r, g, b = r[keep], g[keep], b[keep]
        print(f"{LOG_PREFIX_INFO} ROR 后: {len(xyz):,} 点 (移除 {removed:,} 点)")

    save_merged_pcd(output_path, xyz, r, g, b)

    print(f"{LOG_PREFIX_INFO} 转换完成!")


if __name__ == "__main__":
    main()
