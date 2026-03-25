#!/usr/bin/env python3
"""
功能（track_relative_pose_odom_to_base.py）：
运行时只订阅 `/tf`，持续缓存 odom -> odin1_base_link 的轨迹点与姿态；
你原地自转若干圈后按 `Ctrl+C`，脚本停止订阅并一次性计算：
1) 车体（base_link）旋转中心在 odom 坐标系下的位置；
2) 由 odin1 的 roll/pitch 估计 pitch/roll 补偿，使 base_link 与地面平行；
3) 输出静态变换命令所需参数：
   `--frame-id odin1_base_link --child-frame-id base_link --x --y --z --roll --pitch --yaw`

你关心的“pitch 补偿”来源：
- 通过 tf 中 odom -> odin1_base_link 的四元数计算 odin1 的 roll/pitch；
- 若认为 odom 的 roll/pitch 为 0（与地面平行），则 base_link 的 roll/pitch 目标为 0；
- 因此 odin1 -> base_link 的旋转用“抵消 odin1 的 roll/pitch”构造（yaw 不用于补偿）。

计算方法概述：
1) 轨迹点记录：
   收集每次匹配到的 odom 坐标系下 odin1_base_link 的 (x_i, y_i, z_i) 与四元数 q_i。
2) 旋转中心估计（2D 圆拟合）：
   将 (x_i, y_i) 拟合为圆：x^2 + y^2 = 2*c_x*x + 2*c_y*y + c_0
   得到旋转中心 C = (c_x, c_y)，并令 z_center = 平均 z_i。
3) odin1 -> base_link 平移（刚体外参）：
   对每帧计算向量 v_odom = C - p_i（从 odin1_origin 指向 base_link_center）；
   将其从 odom 表达到 odin1 坐标系：
   v_odin = R(odom->odin1)^{-1} * v_odom  （即 v_odin = R^T * v_odom）
   对 v_odin 求平均，得到静态外参平移 (t_x, t_y, t_z)。
4) odin1 -> base_link 旋转（roll/pitch 补偿）：
   从 q_i 计算 odin1 的平均 (roll_avg, pitch_avg)；
   令 base_link 目标 roll=0, pitch=0，因此将输出旋转 roll=-roll_avg, pitch=-pitch_avg；
   yaw 默认输出 0（你后续若需要 yaw 也可扩展为从 tf 推导）。

注意：
- 此算法默认“自转时基座旋转中心近似固定”，并且主要运动在水平面（pitch/roll 只用于补偿方向）。
- 如果自转不够“纯转”（有明显平移），圆拟合残差会增大，输出会变差。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


# =============================
# 可配置参数（静态变量，统一放在脚本顶部）
# =============================

# TF 订阅与帧名
TF_TOPIC: str = '/tf'
SOURCE_FRAME: str = 'odom'
TARGET_FRAME: str = 'odin1_base_link'

# 订阅队列深度
SUB_QUEUE_DEPTH: int = 200

# pitch/roll 补偿是否参与输出
USE_PITCH_ROLL_COMPENSATION: bool = True

# yaw 输出策略
# - 0: 输出 0（仅补偿 roll/pitch）
# - 1: 输出平均 odin1 yaw（如你想让 yaw 也对齐，可改为 1 并扩展逻辑）
OUTPUT_YAW_MODE: int = 0

# 圆拟合最少点数（不足会报错退出）
MIN_POINTS_FOR_FIT: int = 20

# 输出精度
PRINT_DECIMALS: int = 6

# 自转过程中的采样过滤（可选）
# - 0: 不跳点
# - N: 每 N 帧才记录一次（降低计算量，但减少点数）
SAMPLE_SKIP: int = 0


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_ns: int


def quat_normalize(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """四元数 -> roll/pitch/yaw（rad），采用标准欧拉角公式。"""
    qx, qy, qz, qw = quat_normalize(qx, qy, qz, qw)

    # roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> List[List[float]]:
    """返回旋转矩阵 R，使得 v_parent = R * v_child。"""
    qx, qy, qz, qw = quat_normalize(qx, qy, qz, qw)
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def mat_transpose_3x3(m: List[List[float]]) -> List[List[float]]:
    return [
        [m[0][0], m[1][0], m[2][0]],
        [m[0][1], m[1][1], m[2][1]],
        [m[0][2], m[1][2], m[2][2]],
    ]


def mat_vec_mul_3x3(m: List[List[float]], v: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def fit_circle_2d(xs: List[float], ys: List[float]) -> Tuple[float, float]:
    """
    2D 圆拟合（最小二乘，Kasa 方法）：
    x^2 + y^2 = 2*cx*x + 2*cy*y + c0
    返回 (cx, cy)。
    """
    # 线性最小二乘：A*[cx,cy,c0/2] = b
    # 这里直接构造并用正规方程求解（避免引入 numpy 依赖）
    n = len(xs)
    # 使用 3 参数：cx, cy, k (k=c0/2)
    # 方程：2cx*x + 2cy*y + 1*k = x^2 + y^2
    # A = [2x_i, 2y_i, 1]
    # b = x_i^2 + y_i^2

    # 计算 A^T A 和 A^T b（3x3）
    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3

    for x, y in zip(xs, ys):
        a0 = 2.0 * x
        a1 = 2.0 * y
        a2 = 1.0
        b = x * x + y * y
        a = (a0, a1, a2)
        # ata += a^T a
        for i in range(3):
            atb[i] += a[i] * b
            for j in range(3):
                ata[i][j] += a[i] * a[j]

    # 解 3x3 线性方程：ata * sol = atb
    # 用高斯消元
    # 构造增广矩阵
    aug = [ata[0][:] + [atb[0]], ata[1][:] + [atb[1]], ata[2][:] + [atb[2]]]
    for col in range(3):
        # 找主元
        pivot = col
        for r in range(col + 1, 3):
            if abs(aug[r][col]) > abs(aug[pivot][col]):
                pivot = r
        if abs(aug[pivot][col]) < 1e-12:
            raise RuntimeError("circle fit failed: singular matrix")
        aug[col], aug[pivot] = aug[pivot], aug[col]

        # 归一
        div = aug[col][col]
        for c in range(col, 4):
            aug[col][c] /= div

        # 消元
        for r in range(3):
            if r == col:
                continue
            factor = aug[r][col]
            if abs(factor) < 1e-15:
                continue
            for c in range(col, 4):
                aug[r][c] -= factor * aug[col][c]

    sol0, sol1, _k = aug[0][3], aug[1][3], aug[2][3]
    # 由于我们方程用的是 2cx*x + 2cy*y + 1*k
    # 因此 sol0= cx, sol1= cy
    return sol0, sol1


class TfOdomOdinSelfRotCenterFinder(Node):
    def __init__(self):
        super().__init__('tf_odom_odin1_selfrot_center_finder')
        self.samples: List[PoseSample] = []
        self.initialized: bool = False
        self._cb_counter: int = 0

        self.subscription = self.create_subscription(TFMessage, TF_TOPIC, self.tf_callback, SUB_QUEUE_DEPTH)

        self.get_logger().info(f">>> 运行中：订阅 {TF_TOPIC}，缓存 {SOURCE_FRAME} -> {TARGET_FRAME} 的轨迹点。")
        self.get_logger().info(">>> 运行时请让车辆自转若干圈；完成后按 Ctrl+C 得到最终参数输出。")

    def tf_callback(self, msg: TFMessage):
        self._cb_counter += 1
        if SAMPLE_SKIP > 0 and (self._cb_counter % (SAMPLE_SKIP + 1) != 0):
            return

        for t in msg.transforms:
            if t.header.frame_id != SOURCE_FRAME or t.child_frame_id != TARGET_FRAME:
                continue

            stamp_ns = t.header.stamp.sec * 1_000_000_000 + t.header.stamp.nanosec
            tr = t.transform.translation
            rot = t.transform.rotation
            self.samples.append(PoseSample(
                x=tr.x, y=tr.y, z=tr.z,
                qx=rot.x, qy=rot.y, qz=rot.z, qw=rot.w,
                stamp_ns=int(stamp_ns),
            ))
            self.initialized = True
            break

    def compute_and_print(self):
        if len(self.samples) < MIN_POINTS_FOR_FIT:
            raise RuntimeError(f"Not enough samples for fit: {len(self.samples)} < {MIN_POINTS_FOR_FIT}")

        xs = [s.x for s in self.samples]
        ys = [s.y for s in self.samples]
        zs = [s.z for s in self.samples]

        # 1) 圆拟合得到旋转中心（odom 中）
        cx, cy = fit_circle_2d(xs, ys)
        cz = sum(zs) / len(zs)

        # 2) 计算 odin1 -> base_link 平移：v_odin = R^T * (C - p_odin)
        vxs: List[float] = []
        vys: List[float] = []
        vzs: List[float] = []

        roll_list: List[float] = []
        pitch_list: List[float] = []
        yaw_list: List[float] = []

        C = (cx, cy, cz)
        for s in self.samples:
            # odom->odin1 的旋转矩阵 R，使 v_odom = R * v_odin
            R = quat_to_rot_matrix(s.qx, s.qy, s.qz, s.qw)
            Rt = mat_transpose_3x3(R)  # v_odin = R^T * v_odom
            v_odom = (C[0] - s.x, C[1] - s.y, C[2] - s.z)
            v_odin = mat_vec_mul_3x3(Rt, v_odom)
            vxs.append(v_odin[0])
            vys.append(v_odin[1])
            vzs.append(v_odin[2])

            roll, pitch, yaw = quat_to_rpy(s.qx, s.qy, s.qz, s.qw)
            roll_list.append(roll)
            pitch_list.append(pitch)
            yaw_list.append(yaw)

        tx = sum(vxs) / len(vxs)
        ty = sum(vys) / len(vys)
        tz = sum(vzs) / len(vzs)

        # 3) 旋转补偿：roll/pitch 取平均后取反
        roll_avg = sum(roll_list) / len(roll_list)
        pitch_avg = sum(pitch_list) / len(pitch_list)
        yaw_avg = sum(yaw_list) / len(yaw_list)

        if USE_PITCH_ROLL_COMPENSATION:
            roll_out = -roll_avg
            pitch_out = -pitch_avg
        else:
            roll_out = 0.0
            pitch_out = 0.0

        # yaw 输出：默认 0
        if OUTPUT_YAW_MODE == 1:
            yaw_out = 0.0 if not USE_PITCH_ROLL_COMPENSATION else 0.0  # 先保持 0，便于你稳定验证
            # 若你要 yaw 对齐，我可以再把 yaw_comp 逻辑补全
        else:
            yaw_out = 0.0

        fmt = f"{{:.{PRINT_DECIMALS}f}}"
        x_s = fmt.format(tx)
        y_s = fmt.format(ty)
        z_s = fmt.format(tz)
        roll_s = fmt.format(roll_out)
        pitch_s = fmt.format(pitch_out)
        yaw_s = fmt.format(yaw_out)

        # 按你的最终发布方式输出参数
        print("\n===== static transform params (odom-based self-rot calibration) =====")
        print(f"--frame-id {TARGET_FRAME} --child-frame-id base_link")
        print(f"--x {x_s} --y {y_s} --z {z_s}")
        print(f"--roll {roll_s} --pitch {pitch_s} --yaw {yaw_s}")
        print("==========================================================================\n")


def main():
    rclpy.init()
    node = TfOdomOdinSelfRotCenterFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("检测到 Ctrl+C：停止订阅并开始计算旋转中心与 pitch 补偿...")
    finally:
        # spin 停止后进行计算
        try:
            node.compute_and_print()
        except Exception as e:
            node.get_logger().error(f"计算失败: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

