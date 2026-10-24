#!/mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main/.venv/bin/python3
"""
室内无GNSS环境下左目单目视觉+IMU+轮速计融合定位
数据集：i2Nav-Robot parking01
"""

import cv2
import numpy as np
import yaml
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib import rcParams
import argparse
import math
import sys
from typing import Tuple, List

from wheel_odometry import compute_wheel_odometry
from depth_prior import (
    DynamicDepthScale,
    MetricDepthEdgeFilter,
    estimate_metric_pose,
)

LIGHTGLUE_DIR = Path(__file__).resolve().parent / "LightGlue-main"
if str(LIGHTGLUE_DIR) not in sys.path:
    sys.path.insert(0, str(LIGHTGLUE_DIR))

import torch
from lightglue import LightGlue, SuperPoint

# 设置中文字体（按可用性回退，避免图上出现方框）
rcParams['font.sans-serif'] = [
    'SimHei', 'Noto Sans CJK SC', 'Noto Sans CJK JP',
    'WenQuanYi Zen Hei', 'AR PL UMing CN', 'DejaVu Sans',
]
rcParams['axes.unicode_minus'] = False


class MonoCamera:
    """Camera intrinsics for the left-image monocular VO pipeline."""
    
    def __init__(self, calibration_file: str):
        """
        从标定文件加载相机参数
        
        Args:
            calibration_file: calibration.yaml 路径
        """
        with open(calibration_file, 'r') as f:
            calib = yaml.safe_load(f)
        
        # 左目相机参数
        left_intrinsics = calib['camera']['left']['intrinsic']
        self.fx_left = left_intrinsics[0]
        self.fy_left = left_intrinsics[1]
        self.cx_left = left_intrinsics[2]
        self.cy_left = left_intrinsics[3]
        
        # 左目畸变系数
        self.dist_left = np.array(calib['camera']['left']['distortions'])
        
        # Left-camera intrinsic matrix
        self.K_left = np.array([
            [self.fx_left, 0, self.cx_left],
            [0, self.fy_left, self.cy_left],
            [0, 0, 1]
        ])
        print(
            f"左目相机参数加载完成: fx={self.fx_left:.2f}, "
            f"fy={self.fy_left:.2f}, cx={self.cx_left:.2f}, cy={self.cy_left:.2f}"
        )


class MonoVisualOdometry:
    """Left-camera monocular VO using SuperPoint and LightGlue."""
    
    def __init__(
        self,
        camera: MonoCamera,
        scale: float = 0.5,
        depth_filter: MetricDepthEdgeFilter = None,
        translation_source: str = "wheel",
        dynamic_scale: DynamicDepthScale = None,
        wheel_scale_from_integral: bool = False,
    ):
        """
        Initialize left-camera monocular VO.
        
        Args:
            camera: Camera calibration containing the left-camera parameters.
            scale: Image processing scale factor
            wheel_scale_from_integral: 用轮速的梯形积分距离（与 EKF 预测所用的
                位移完全一致）为单目平移定标，而不是瞬时轮速 × Δt。后者在加减速
                段会带来约 a·Δt/2 ≈ 6% 的尺度偏差。
        """
        self.camera = camera
        self.scale = scale
        self.depth_filter = depth_filter
        self.translation_source = translation_source
        self.dynamic_scale = dynamic_scale
        self.wheel_scale_from_integral = wheel_scale_from_integral
        if translation_source in ("depth", "depth_dynamic") and depth_filter is None:
            raise ValueError("metric-depth VO requires a Depth Anything model")
        if translation_source == "depth_dynamic" and dynamic_scale is None:
            raise ValueError("dynamic depth VO requires a scale estimator")
        self.prev_depth_edges = None
        self.prev_depth = None
        self.last_match_count = 0
        self.last_pose_inliers = 0
        self.last_raw_forward = 0.0
        self.last_raw_lateral = 0.0
        self.last_scale_factor = 1.0
        
        # Scaled intrinsics
        self.K_left_scaled = self.camera.K_left.copy()
        self.K_left_scaled[0, :] *= scale
        self.K_left_scaled[1, :] *= scale
        self.K_left_scaled[2, 2] = 1.0
        
        self._build_undistortion_map(scale)
        
        weight_paths = (
            Path(__file__).resolve().parent / "superpoint_lightglue" / "superpoint_backbone.pth",
            LIGHTGLUE_DIR / "lightglue" / "weights" / "superpoint_lightglue.pth",
        )
        missing_weights = [str(path) for path in weight_paths if not path.is_file()]
        if missing_weights:
            raise FileNotFoundError(
                "LightGlue model weights are missing: " + ", ".join(missing_weights)
            )

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.detector = SuperPoint(max_num_keypoints=2048).eval().to(self.device)
        self.matcher = LightGlue(features="superpoint").eval().to(self.device)
        print(f"LightGlue特征匹配已启用，设备: {self.device}, 最大关键点数: 2048")
        
        # Features from the previous left image.
        self.prev_features_left = None

    def _build_undistortion_map(self, scale: float):
        """Build an undistortion map for the left camera only."""
        img_size = (int(1600 * scale), int(1200 * scale))
        K_left_scaled = self.camera.K_left.copy()
        K_left_scaled[:2, :] *= scale
        self.map1x, self.map1y = cv2.initUndistortRectifyMap(
            self.camera.K_left, self.camera.dist_left, None,
            K_left_scaled, img_size, cv2.CV_32FC1
        )
        self.K_left_scaled = K_left_scaled
        print(f"图像去畸变完成, 缩放后内参: fx={K_left_scaled[0,0]:.1f}, cx={K_left_scaled[0,2]:.1f}")

    def _undistort(self, img_left: np.ndarray) -> np.ndarray:
        """Undistort a left-camera image."""
        left_undist = cv2.remap(img_left, self.map1x, self.map1y, cv2.INTER_LINEAR)
        return left_undist
        
    def load_image(self, path: str) -> np.ndarray:
        """加载图像并缩放"""
        img = cv2.imread(path)
        if img is None:
            raise FileNotFoundError(f"无法读取图像: {path}")
        
        if self.scale != 1.0:
            img = cv2.resize(img, None, fx=self.scale, fy=self.scale)
        
        return img
    
    def _extract_features(self, image: np.ndarray) -> dict:
        """Extract SuperPoint keypoints and descriptors from a BGR image."""
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(rgb.copy()).permute(2, 0, 1).float().div_(255.0)
        tensor = tensor.unsqueeze(0).to(self.device)
        with torch.inference_mode():
            return self.detector.extract(tensor)

    def _match_features(self, features0: dict, features1: dict) -> np.ndarray:
        """Match two extracted feature sets and return keypoint index pairs."""
        with torch.inference_mode():
            result = self.matcher({"image0": features0, "image1": features1})
        return result["matches"][0].detach().cpu().numpy().astype(np.int32, copy=False)

    @staticmethod
    def _keypoints_xy(features: dict) -> np.ndarray:
        return features["keypoints"][0].detach().cpu().numpy()

    def process(
        self,
        img_left_path: str,
        dt: float,
        wheel_speed: float,
        wheel_distance: float = None,
    ) -> Tuple[bool, float, float, float, int]:
        """Estimate frame motion using wheel-scaled VO or metric-depth PnP."""
        img_left = self.load_image(img_left_path)
        img_left = self._undistort(img_left)
        features_left = self._extract_features(img_left)
        depth = (
            self.depth_filter.infer_depth(img_left)
            if self.depth_filter is not None
            else None
        )
        depth_edges = self.depth_filter.edge_mask(depth) if depth is not None else None
        self.last_match_count = 0
        self.last_pose_inliers = 0
        self.last_raw_forward = 0.0
        self.last_raw_lateral = 0.0
        self.last_scale_factor = (
            self.dynamic_scale.scale if self.dynamic_scale is not None else 1.0
        )
        if self.prev_features_left is None:
            self.prev_features_left = features_left
            self.prev_depth_edges = depth_edges
            self.prev_depth = depth
            return False, 0.0, 0.0, 0.0, 0

        previous_features = self.prev_features_left
        matches = self._match_features(previous_features, features_left)
        self.prev_features_left = features_left
        previous_depth_edges = self.prev_depth_edges
        self.prev_depth_edges = depth_edges
        previous_depth = self.prev_depth
        self.prev_depth = depth
        self.last_match_count = len(matches)
        if len(matches) < 8 or dt <= 0:
            return False, 0.0, 0.0, 0.0, len(matches)

        previous_points = self._keypoints_xy(previous_features)
        current_points = self._keypoints_xy(features_left)
        points0 = previous_points[matches[:, 0]].astype(np.float32)
        points1 = current_points[matches[:, 1]].astype(np.float32)
        if self.depth_filter is not None:
            matches, _ = self.depth_filter.filter_matches(
                matches,
                points0,
                points1,
                previous_depth_edges,
                depth_edges,
            )
            if len(matches) < 8:
                return False, 0.0, 0.0, 0.0, len(matches)
            points0 = previous_points[matches[:, 0]].astype(np.float32)
            points1 = current_points[matches[:, 1]].astype(np.float32)
            self.last_match_count = len(matches)

        if self.translation_source in ("depth", "depth_dynamic"):
            try:
                rotation, translation, inliers = estimate_metric_pose(
                    points0,
                    points1,
                    previous_depth,
                    self.K_left_scaled,
                    max_depth=self.depth_filter.model.max_depth,
                )
            except ValueError:
                return False, 0.0, 0.0, 0.0, 0
            camera_motion = -rotation.T @ translation
            self.last_pose_inliers = inliers
            forward = float(camera_motion[2])
            lateral = float(camera_motion[0])
            self.last_raw_forward = forward
            self.last_raw_lateral = lateral
            yaw_delta = float(math.atan2(rotation[0, 2], rotation[2, 2]))
            if not np.isfinite([forward, lateral, yaw_delta]).all():
                return False, 0.0, 0.0, 0.0, inliers
            if self.translation_source == "depth_dynamic":
                depth_distance = float(np.hypot(forward, lateral))
                scale_reference_distance = (
                    abs(float(wheel_speed) * dt)
                    if wheel_distance is None
                    else float(wheel_distance)
                )
                calibrated = self.dynamic_scale.update(
                    depth_distance, scale_reference_distance, inliers
                )
                self.last_scale_factor = self.dynamic_scale.scale
                if not calibrated:
                    return False, 0.0, 0.0, 0.0, inliers
                forward *= self.last_scale_factor
                lateral *= self.last_scale_factor
            return True, forward, lateral, yaw_delta, inliers

        essential, mask = cv2.findEssentialMat(
            points0,
            points1,
            self.K_left_scaled,
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1.0,
        )
        if essential is None or mask is None:
            return False, 0.0, 0.0, 0.0, len(matches)

        inliers, rotation, translation, _ = cv2.recoverPose(
            essential, points0, points1, self.K_left_scaled, mask=mask
        )
        if inliers < 8:
            return False, 0.0, 0.0, 0.0, int(inliers)
        self.last_pose_inliers = int(inliers)

        camera_motion = -rotation.T @ translation.reshape(3)
        direction_norm = float(np.linalg.norm(camera_motion))
        if direction_norm < 1e-8:
            return False, 0.0, 0.0, 0.0, int(inliers)

        travel_distance = abs(float(wheel_speed) * dt)
        if self.wheel_scale_from_integral and wheel_distance is not None:
            # 与 EKF 预测所用位移一致的梯形积分距离
            travel_distance = float(wheel_distance)
        camera_motion *= travel_distance / direction_norm
        forward = float(camera_motion[2])
        lateral = float(camera_motion[0])
        self.last_raw_forward = forward
        self.last_raw_lateral = lateral
        yaw_delta = float(math.atan2(rotation[0, 2], rotation[2, 2]))
        if not np.isfinite([forward, lateral, yaw_delta]).all():
            return False, 0.0, 0.0, 0.0, int(inliers)

        return True, forward, lateral, yaw_delta, int(inliers)


class PlanarEKF:
    """Plane EKF [x, y, yaw, speed, gyro_bias (, C_lane)] with chi-squared gating.

    lane_heading=True 时增广第 6 个状态 C_lane：车道网方位常数（弧度），
    观测模型 ψ + α ≡ C_lane (mod 90°)。C_lane 为场地级常量（Q=0），
    它使车道观测在里程计系内自洽收敛，无需任何绝对信息即可消除偏航漂移；
    绝对定位模式（--lane-abs-cal-deg）则把 C 固定为一次性设施标定值。
    """

    # Chi-squared 95% quantiles for different DOF
    CHI2_95 = {1: 3.841, 2: 5.991, 3: 7.815, 5: 11.070}

    def __init__(self, wheel_speed_noise: float = 0.15,
                 visual_position_noise: float = 0.5,
                 protect_yaw_from_visual: bool = True,
                 lane_heading: bool = False,
                 lane_C_prior_deg: float = 15.0,
                 yaw_q: float = 0.003):
        """
        Args:
            wheel_speed_noise: 轮速计噪声
            visual_position_noise: 视觉位置噪声
            protect_yaw_from_visual: 视觉位置更新不改动 yaw / 陀螺零偏状态
            lane_heading: 增广车道航向观测通道（第 6 状态 C_lane）
            yaw_q: 偏航过程噪声谱 Q[2,2]（rad²/s）。默认 0.003 与归档基线逐位一致；
                   P1-1 实验用 1e-6（≈0.06°/√s 随机游走量级）
        """
        # 状态向量 [x, y, yaw, speed, gyro_bias, (C_lane)]
        self.n = 6 if lane_heading else 5
        self.x = np.zeros(self.n)
        self.lane_heading = lane_heading
        self.yaw_q = yaw_q
        
        # 协方差矩阵
        p_diag = [1.0, 1.0, 0.3, 1.0, 1e-6]
        if lane_heading:
            p_diag.append(np.radians(lane_C_prior_deg) ** 2)  # C_lane 先验
        self.P = np.diag(p_diag)  # gyro_bias极小不确定性
        
        # 噪声参数
        self.wheel_speed_noise = wheel_speed_noise
        self.visual_position_noise = visual_position_noise
        self.protect_yaw_from_visual = protect_yaw_from_visual

        # 统计信息
        self.nis_values = []      # 全部 NIS（视觉 + 轮速）
        self.visual_nis = []      # 仅视觉位置更新的 NIS
        self.wheel_nis = []       # 仅轮速更新的 NIS
        self.lane_nis = []        # 仅车道航向更新的 NIS
        self.outliers = 0         # 视觉被门控拒绝的次数
        self.total_updates = 0    # 视觉更新总次数
        self.wheel_gate_rejects = 0
        self.lane_updates = 0     # 车道航向更新通过次数
        self.lane_rejects = 0     # 车道航向 NIS 拒绝次数

    def nis_gate(self, innovation: np.ndarray, S: np.ndarray, dof: int):
        """Chi-squared gating.

        Returns (passed, nis). NIS 按来源分开记录：轮速更新的 innovation 在结构上
        接近零（predict 里已把 x[3] 置为轮速），与视觉 NIS 混在一个数组里会把视觉
        的真实 NIS 稀释成假的 ~0，因此必须分道统计。
        """
        nis = float(innovation.T @ np.linalg.inv(S) @ innovation)
        self.nis_values.append(nis)
        threshold = self.CHI2_95.get(dof, 3.841)
        return nis < threshold, nis

    def set_initial_yaw(self, yaw: float):
        """Set initial yaw angle (typically from ground-truth or alignment)."""
        self.x[2] = self.wrap_angle(yaw)

    def set_initial_gyro_bias(self, bias_z: float, bias_std: float = 1e-4):
        """Initialize the gyro-bias state from stationary alignment.

        The bias state is kept nearly locked (small P, small Q in predict), so
        setting it to the stationary mean directly removes long-horizon yaw drift.
        """
        self.x[4] = bias_z
        self.P[4, 4] = bias_std ** 2
    
    def predict(self, dt: float, gyro_z: float, wheel_speed: float):
        """
        预测步骤
        
        Args:
            dt: 时间步长
            gyro_z: Z轴角速度
            wheel_speed: 轮速计速度
        """
        x, y, yaw, speed, bias = self.x[:5]
        
        # 校正后的角速度
        yaw_rate = gyro_z - bias
        
        # 状态预测
        self.x[0] = x + speed * np.cos(yaw) * dt
        self.x[1] = y + speed * np.sin(yaw) * dt
        self.x[2] = self.wrap_angle(yaw + yaw_rate * dt)
        self.x[3] = wheel_speed
        # （C_lane 为场地常量：无动力学、无过程噪声）
        
        # 雅可比矩阵
        F = np.eye(self.n)
        F[0, 2] = -speed * np.sin(yaw) * dt
        F[0, 3] = np.cos(yaw) * dt
        F[1, 2] = speed * np.cos(yaw) * dt
        F[1, 3] = np.sin(yaw) * dt
        F[2, 4] = -dt
        
        # 过程噪声
        q_diag = [0.01, 0.01, self.yaw_q, self.wheel_speed_noise**2, 1e-10]
        if self.lane_heading:
            q_diag.append(0.0)          # 设施常数不随时间随机游走
        Q = np.diag(q_diag) * max(dt, 1e-3)
        
        # 协方差预测
        self.P = F @ self.P @ F.T + Q

    def update_visual_position(self, xy_meas: np.ndarray) -> bool:
        """Fuse visual position (x, y) only; yaw stays driven by IMU gyro.

        Args:
            xy_meas: [x, y] position measurement in the world frame
        Returns:
            True if the measurement passed the chi-squared gate.
        """
        H = np.zeros((2, self.n))
        H[0, 0] = 1.0  # x
        H[1, 1] = 1.0  # y

        innovation = xy_meas - H @ self.x

        R = np.diag([
            self.visual_position_noise**2,
            self.visual_position_noise**2,
        ])

        S = H @ self.P @ H.T + R

        # NIS chi-squared gate (dof=2)
        self.total_updates += 1
        passed, nis = self.nis_gate(innovation, S, dof=2)
        self.visual_nis.append(nis)
        if not passed:
            self.outliers += 1
            return False

        K = self.P @ H.T @ np.linalg.inv(S)

        # 冗余观测只修正其直接观测的状态。单目VO的平移既由轮速定标、方向又用
        # 本EKF的yaw旋回世界系，因此与预测高度重复、不含独立定位信息；若允许它
        # 经由 P 的 yaw-x / yaw-y 互协方差反算 yaw，长轨迹上偏航会被拉飞
        # （实测 17min：位置 1.17m→30.8m，偏航 1.9°→54.7°）。
        if self.protect_yaw_from_visual:
            keep = np.zeros((self.n, 1), dtype=bool)
            keep[[0, 1], 0] = True          # 只修正 x, y
            K = K * keep

        self.x = self.x + K @ innovation
        self.x[2] = self.wrap_angle(self.x[2])

        # Joseph 形式，保证 K 行被置零后协方差仍对称半正定
        A = np.eye(self.n) - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        return True

    def update_wheel_speed(self, wheel_speed: float):
        """
        Wheel speed update with chi-squared gating.
        
        Args:
            wheel_speed: 轮速计速度 (m/s)
        """
        H = np.zeros((1, self.n))
        H[0, 3] = 1.0
        
        # Innovation
        innovation = np.array([wheel_speed - self.x[3]])
        
        # Observation noise
        R = np.array([[self.wheel_speed_noise**2]])
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # NIS chi-squared gate (dof=1, threshold=3.841)
        passed, nis = self.nis_gate(innovation, S, dof=1)
        self.wheel_nis.append(nis)
        if not passed:
            self.wheel_gate_rejects += 1
            return
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # 同上：predict 已把 x[3] 直接置为轮速，该观测近乎冗余，
        # 只修正速度状态，避免经 P[2,3]/P[4,3] 污染 yaw 与零偏。
        if self.protect_yaw_from_visual:
            keep = np.zeros((self.n, 1), dtype=bool)
            keep[3, 0] = True
            K = K * keep

        # State update
        self.x = self.x + (K @ innovation).flatten()
        
        # Covariance update
        A = np.eye(self.n) - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T

    def update_lane_heading(self, alpha_rad: float, r_deg: float,
                            abs_C_rad=None) -> bool:
        """车道线航向观测（P0-2）。与视觉位置更新本质不同：
        它是偏航的**独立**观测源（重力+标定几何，不含轮速/IMU 信息），
        因此允许直接修正 yaw / gyro 零偏 / C_lane，不受 protect_yaw 约束。

        观测模型：
          grid 模式（abs_C_rad=None，C 为第 6 状态）：ψ + α − C ≡ 0 (mod 90°)
            —— 里程计系内自洽，不需要任何绝对信息，作用是**消除偏航漂移**。
          abs 模式（给定设施标定常数 C0）：ψ_meas = C0 − α (mod 90°)，
            创新用 ±180° 环绕（初始象限由标定/象限输入定，NIS 门拒绝大跳变）。
        Args:
            alpha_rad: 车道族方向角 α（相对相机前向，弧度，(-π/2, π/2]）
            r_deg: 该次观测的标准差（度）
        """
        if self.lane_heading and abs_C_rad is None:
            # 观测模型 h(x) = ψ + α − C ≡ 0 (mod 90°)；innovation = z − h = −(ψ+α−C)
            resid = self.wrap_mod(self.x[5] - alpha_rad - self.x[2], np.pi / 2)
            H = np.zeros((1, self.n))
            H[0, 2], H[0, 5] = 1.0, -1.0
        elif abs_C_rad is not None:
            # ψ_meas = C0 − α (mod 90°)。创新同样按 mod-90 收缩：初始化时象限 k 已由
            # 一次性输入与"前进方向族"门选定，其后正交族翻转（通道里看到库位线）
            # 折回 ≈0，不污染航向；跨族跳变本就该由网格不变性吸收而非 NIS 拒绝。
            resid = self.wrap_mod(abs_C_rad - alpha_rad - self.x[2], np.pi / 2)
            H = np.zeros((1, self.n))
            H[0, 2] = 1.0
        else:
            return False
        innovation = np.array([resid])
        R = np.array([[np.radians(r_deg) ** 2]])
        S = H @ self.P @ H.T + R

        passed, nis = self.nis_gate(innovation, S, dof=1)
        self.lane_nis.append(nis)
        self.lane_updates_total = getattr(self, "lane_updates_total", 0) + 1
        if not passed:
            self.lane_rejects += 1
            return False

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ innovation).flatten()
        self.x[2] = self.wrap_angle(self.x[2])
        A = np.eye(self.n) - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.lane_updates += 1
        return True

    @staticmethod
    def wrap_mod(angle: float, period: float) -> float:
        """网格角折回 [-period/2, period/2)（车道正交族 mod-90° 用）。"""
        return (angle + period / 2.0) % period - period / 2.0
    
    @staticmethod
    def wrap_angle(angle: float) -> float:
        """角度归一化到 [-pi, pi]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi


def load_imu_data(imu_file: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load the ADIS16465 yaw-rate channel used by the planar EKF.
    
    The raw data uses a custom scaling (≈200× smaller than SI units).
    col1-3: gyro_x, gyro_y, gyro_z  → multiply by 200 → rad/s
    
    Returns:
        timestamps: (N,)
        gyro_z:     yaw rate in rad/s (left-turn positive, already signed)
    """
    SI_SCALE = 200.0
    data = np.loadtxt(imu_file)
    timestamps = data[:, 0]
    gyro_z = data[:, 3] * SI_SCALE   # gyro_z already left-turn-positive
    return timestamps, gyro_z


def load_imu_full(imu_file: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load full 3-axis ADIS16465 IMU data scaled to SI units.

    Returns:
        timestamps: (N,)
        gyro:       (N, 3) gx, gy, gz in rad/s
        accel:      (N, 3) ax, ay, az in m/s^2 (specific force; FRD frame)
    """
    SI_SCALE = 200.0
    data = np.loadtxt(imu_file)
    timestamps = data[:, 0]
    gyro = data[:, 1:4] * SI_SCALE
    accel = data[:, 4:7] * SI_SCALE
    return timestamps, gyro, accel


def stationary_alignment(imu_file: str,
                         gravity: float = 9.80665,
                         accel_tol: float = 0.15,
                         gyro_tol: float = 0.02,
                         motion_hold: float = 0.3) -> dict:
    """IMU-based stationary (power-on) alignment.

    Detects the initial stationary interval from accelerometer magnitude
    stability and gyroscope quiescence, estimates the three gyro biases as the
    mean gyro over that window, and solves the initial roll/pitch from the mean
    gravity vector. ADIS16465 has no magnetometer, so absolute yaw is NOT
    observable from a static pose: the navigation frame heading is set to 0
    (vehicle-odometry frame).

    Returns dict with: t_motion_start, skip_seconds, gyro_bias (3,), roll_deg,
    pitch_deg, n_static.
    """
    t, gyro, accel = load_imu_full(imu_file)
    w_norm = np.linalg.norm(gyro, axis=1)
    a_norm = np.linalg.norm(accel, axis=1)
    static = (np.abs(a_norm - gravity) < accel_tol) & (w_norm < gyro_tol)

    # First time after a 5s warm-up whose non-static state persists; that marks
    # the end of the initial stationary run (vehicle start of motion).
    i_end = len(t) - 1
    for i in range(len(t)):
        if t[i] - t[0] <= 5.0 or static[i]:
            continue
        j = np.searchsorted(t, t[i] + motion_hold)
        if j > i and (~static[i:j]).mean() > 0.5:
            i_end = i
            break

    seg_gyro = gyro[:i_end]
    seg_accel = accel[:i_end]
    bias = seg_gyro.mean(axis=0)
    a_mean = seg_accel.mean(axis=0)

    # FRD frame: static specific force ≈ (0, 0, -g). Tilt from the dominant axis.
    down = -a_mean
    roll_deg = np.degrees(np.arctan2(down[1], down[2]))
    pitch_deg = np.degrees(np.arctan2(-down[0], np.hypot(down[1], down[2])))

    return {
        "t_motion_start": float(t[i_end]),
        "skip_seconds": float(t[i_end] - t[0]),
        "gyro_bias": bias,
        "roll_deg": float(roll_deg),
        "pitch_deg": float(pitch_deg),
        "n_static": int(i_end),
    }


def load_wheel_speed_data(
    odo_file: str, track_width_m: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Return timestamps and mean forward speed.

    左右轮速差也能给出偏航角速度，但实测它与真值积分后偏差达 208°/17min
    （Ranger MINI3 四轮独立转向，差速模型不成立），故不加载该通道。
    """
    data = np.loadtxt(odo_file)
    timestamps = data[:, 0]
    wheel_speed, _yaw_rate = compute_wheel_odometry(data[:, 1:5], track_width_m)
    return timestamps, wheel_speed


def load_image_timestamps(ts_file: str) -> dict:
    """Read the per-frame acquisition times written by extract_stereo_from_bag.py.

    File format: one "<frame_name> <t_rel>" pair per line, where t_rel is seconds
    since the first IMU sample in the bag. Because the exported IMU txt starts at
    the same physical sample, absolute time = imu_times[0] + t_rel.
    """
    stamps = {}
    with open(ts_file) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            name, t = line.split()
            stamps[name] = float(t)
    return stamps


def interpolate_data(values: np.ndarray, src_times: np.ndarray, dst_times: np.ndarray) -> np.ndarray:
    """插值数据到目标时间戳"""
    return np.interp(dst_times, src_times, values)


def integrate_wheel_distance(
    start_time: float,
    end_time: float,
    odo_times: np.ndarray,
    wheel_speed: np.ndarray,
) -> float:
    """Integrate absolute wheel speed over an image-frame interval."""
    if end_time <= start_time:
        return 0.0
    inside = (odo_times > start_time) & (odo_times < end_time)
    times = np.concatenate(([start_time], odo_times[inside], [end_time]))
    speeds = np.interp(times, odo_times, wheel_speed)
    return float(np.trapz(np.abs(speeds), times))


def run_fusion(args):
    """运行融合算法"""
    
    # 加载相机参数
    camera = MonoCamera(args.calibration)
    
    depth_filter = None
    if args.depth_checkpoint or args.vo_translation_source in (
        "depth",
        "depth_dynamic",
    ):
        checkpoint = args.depth_checkpoint or str(
            Path(__file__).resolve().parent
            / "depth_anything_v2_metric_hypersim_vits.pth"
        )
        depth_filter = MetricDepthEdgeFilter(
            checkpoint=checkpoint,
            input_size=args.depth_input_size,
            edge_threshold=args.depth_edge_threshold,
            edge_radius=args.depth_edge_radius,
        )

    dynamic_scale = None
    if args.vo_translation_source == "depth_dynamic":
        dynamic_scale = DynamicDepthScale(
            window_size=args.depth_scale_window,
            min_samples=args.depth_scale_min_samples,
            min_wheel_distance=args.depth_scale_min_wheel_distance,
            min_depth_distance=args.depth_scale_min_depth_distance,
            min_inliers=args.depth_scale_min_inliers,
            min_scale=args.depth_scale_min_factor,
            max_scale=args.depth_scale_max_factor,
        )

    # Initialize left-camera-only monocular visual odometry.
    vo = MonoVisualOdometry(
        camera,
        scale=args.image_scale,
        depth_filter=depth_filter,
        translation_source=args.vo_translation_source,
        dynamic_scale=dynamic_scale,
        wheel_scale_from_integral=args.vo_scale_integral,
    )
    
    # 初始化EKF
    ekf = PlanarEKF(
        wheel_speed_noise=args.wheel_speed_noise,
        visual_position_noise=args.visual_position_noise,
        protect_yaw_from_visual=not args.visual_update_yaw,
        lane_heading=args.lane_heading,
        yaw_q=args.yaw_q,
    )
    
    # 加载IMU和轮速计数据
    imu_times, gyro_z = load_imu_data(args.imu_file)
    odo_times, wheel_speed = load_wheel_speed_data(
        args.odo_file, args.wheel_track_width
    )
    
    # IMU 静止对准（上电对准）：检测运动起始、估计陀螺零偏与横滚/俯仰
    align = stationary_alignment(args.imu_file)
    print("IMU静止对准结果:")
    print(f"  运动起始时刻: {align['t_motion_start']:.3f} (上电后 {align['skip_seconds']:.2f}s)")
    print(f"  陀螺零偏 (rad/s): gx={align['gyro_bias'][0]:.5f} gy={align['gyro_bias'][1]:.5f} gz={align['gyro_bias'][2]:.5f}")
    print(f"  gz零偏 = {np.degrees(align['gyro_bias'][2]):.4f} °/s")
    print(f"  初始姿态: roll={align['roll_deg']:.2f}° pitch={align['pitch_deg']:.2f}°")

    # ===== P0-2 车道航向观测通道（--lane-heading 开启；默认关闭=与既有基线逐位一致） =====
    lane_observer = None
    lane_imu_t = lane_roll = lane_pitch = None
    lane_abs_C = None
    if args.lane_heading:
        import lane_heading as _lh
        _it, _gy, _ac = load_imu_full(args.imu_file)
        lane_imu_t = _it
        lane_roll, lane_pitch = _lh.run_attitude_track(
            _it, _gy, _ac, align["roll_deg"], align["pitch_deg"],
            tau=args.lane_tau, wheel_times=odo_times, wheel_speed=wheel_speed)
        lane_observer = _lh.LaneHeadingObserver(args.calibration)
        lane_observer.GATE_SPREAD = args.lane_gate_spread
        if args.lane_abs_cal_deg is not None:
            lane_abs_C = math.radians(args.lane_abs_cal_deg)
            print(f"  [车道航向] 绝对模式：设施常数 C0={args.lane_abs_cal_deg:+.3f}°"
                  f"（场地级一次性标定），初始象限 k={args.lane_quadrant}"
                  f"{'，由起始静止段观测初始化绝对偏航' if args.lane_init_from_obs else ''}")
        else:
            print("  [车道航向] 自洽模式：C_lane 为增广状态，全程免绝对信息，"
                  "作用是消除偏航漂移（mod-90 网格锁定）")
        print(f"  [车道航向] 在线姿态互补滤波 tau={args.lane_tau:.0f}s；"
              f"观测门限 frac>=0.80 & spread<={args.lane_gate_spread:.2f}°；"
              f"R={('自适应[0.2,%.2f]°' % args.lane_gate_spread) if args.lane_adaptive_R else '%.2f°' % args.lane_noise_deg}")

    if args.init_method == "groundtruth":
        # 参考模式：仅初始 yaw 取自真值（用于对照实验）；起始帧仍由运动起始决定
        gt = np.loadtxt(args.groundtruth)
        init_yaw = np.radians(gt[0, 9])
        print(f"  [真值对照] 初始偏航: {np.degrees(init_yaw):.1f}°")
    else:
        # 部署模式：无磁力计，静态不可观测绝对航向 → 里程计系 yaw=0；
        # 零偏由静止对准设置；起始帧由运动起始自动确定，全程不读取真值。
        init_yaw = 0.0
        print("  [IMU对准] 初始偏航=0°(里程计系)，起始帧由静止对准自动确定")
    ekf.set_initial_yaw(init_yaw)
    if args.apply_gyro_bias:
        ekf.set_initial_gyro_bias(float(align["gyro_bias"][2]))
        print("  已应用静止陀螺零偏修正")
    else:
        print("  未应用陀螺零偏修正（静止对准结果仅作报告）")

    # 获取图像文件列表（已在提取时按每3帧采样）
    left_files = sorted(Path(args.left_dir).glob("frame_*.png"))
    if not left_files:
        raise RuntimeError(f"左目图像目录中没有 frame_*.png 文件: {args.left_dir}")

    # 真实采集时间戳（由 extract_stereo_from_bag.py 从 bag 头部读出）
    ts_file = Path(args.timestamps_file) if args.timestamps_file else \
        Path(args.left_dir) / 'timestamps.txt'
    stamps = load_image_timestamps(ts_file) if ts_file.exists() else None
    t_motion = align['t_motion_start']

    if stamps:
        # 按真实时间排序并过滤掉静止段：不假设恒定帧率
        pairs = sorted((imu_times[0] + stamps[p.name], p)
                       for p in left_files if p.name in stamps)
        pairs = [(t, p) for t, p in pairs if t >= t_motion][:args.max_frames]
        if not pairs:
            raise RuntimeError("按真实时间戳筛选后没有可用图像")
        image_times = np.array([t for t, _ in pairs])
        left_files = [p for _, p in pairs]
        n_frames = len(left_files)
        step = np.diff(image_times)
        print(f"\n处理 {n_frames} 帧左目图像（单目 VO，bag 内真实时间戳）")
        print(f"  时间跨度 {image_times[0]:.3f} ~ {image_times[-1]:.3f} s"
              f"（{image_times[-1] - image_times[0]:.1f} s）")
        print(f"  实测帧间隔 {step.mean() * 1000:.1f} ms → {1 / step.mean():.4f} Hz")
    else:
        # 回退：无 timestamps.txt 时按恒定帧率合成（精度明显更差，建议提取时生成）
        i0 = int(round((t_motion - imu_times[0]) * args.image_fps))
        left_files = left_files[i0:i0 + args.max_frames]
        n_frames = len(left_files)
        if n_frames == 0:
            raise RuntimeError("没有可处理的左目图像")
        print(f"\n仅处理 {n_frames} 帧左目图像"
              f"（按 {args.image_fps} Hz 合成时间戳，建议改用 timestamps.txt）")
        image_times = imu_times[0] + (i0 + np.arange(n_frames)) / args.image_fps
    
    # 插值轮速到图像时刻（供VO定标与结果输出）；EKF预测按IMU率子步另行插值
    wheel_speed_interp = interpolate_data(wheel_speed, odo_times, image_times)
    
    # 融合结果
    results = []
    vo_motion_results = []
    vo_positions = []
    vo_success_count = 0
    cov_peak = np.zeros(3)   # 全程最大的位置/偏航 1σ（用于协方差自洽性检查）
    # Relative-VO anchor: measurement = EKF pos at last accepted frame
    # plus VO world-motion accumulated since that frame. Yaw for rotating
    # camera-frame motion uses the IMU-driven EKF heading (VO yaw is unreliable).
    anchor = np.array([ekf.x[0], ekf.x[1]])
    vo_accum = np.zeros(2)
    # 车道航向通道状态
    lane_buf = []        # mod-90 网格一致性缓冲（防起步/跨道时大 innovation 拉飞偏航）
    lane_records = []    # (t, roll, pitch, alpha, spread, frac, nseg, gate_ok, accepted, nis)
    lane_inited = False  # 绝对模式：初始偏航是否已由首帧有效观测设定

    prev_time = image_times[0]

    # —— P0-2 绝对模式初始化（部署语义：开机先看车道线换系，只认"前进方向族"）——
    def _lane_abs_init(alpha_rad, t_abs, src):
        """由一个有效车道观测初始化绝对偏航，并把历史输出绕上电原点整体旋转换系。
        只接受 |α|≤45° 的观测：α≈90° 意味着看到的是垂线族（库位内对着库位线），
        用它换系会把整车航向锁到错误象限（冒烟测试实测：parking01 静止段看到的
        恰是垂线族，α≈−88.6°）。
        换系公式：ψ_new = C0 − α + k·90°；Δψ = ψ_new − ψ_odo；
        历史 (x,y) 绕原点旋转 Δψ、yaw 加 Δψ —— 部署合理性的根据：上电位置就是
        场地注册原点（充电桩/固定车位），位置先验 (0,0) 与绝对系共用。"""
        nonlocal lane_inited
        psi_new = PlanarEKF.wrap_angle(lane_abs_C - alpha_rad
                                       + math.radians(90.0 * args.lane_quadrant))
        dpsi = float(np.arctan2(math.sin(psi_new - ekf.x[2]),
                                math.cos(psi_new - ekf.x[2])))
        cc, ss = math.cos(dpsi), math.sin(dpsi)
        Rp = np.array([[cc, -ss], [ss, cc]])
        px, py = ekf.x[0], ekf.x[1]
        ekf.x[0], ekf.x[1] = cc * px - ss * py, ss * px + cc * py
        ekf.x[2] = psi_new
        ekf.P[2, 2] = math.radians(1.0) ** 2
        ekf.P[0:2, :] = Rp @ ekf.P[0:2, :]
        ekf.P[:, 0:2] = ekf.P[:, 0:2] @ Rp.T
        for _r in results:
            _rx, _ry = _r["x"], _r["y"]
            _r["x"], _r["y"] = cc * _rx - ss * _ry, ss * _rx + cc * _ry
            _r["yaw"] = PlanarEKF.wrap_angle(_r["yaw"] + dpsi)
        anchor[:] = ekf.x[0:2]
        lane_inited = True
        print(f"  [车道航向] 绝对偏航初始化({src}, t={t_abs - image_times[0]:.0f}s): "
              f"ψ={np.degrees(psi_new):+.2f}° (k={args.lane_quadrant}, "
              f"α={np.degrees(alpha_rad):+.2f}°)，历史换系 {np.degrees(dpsi):+.1f}°")

    if (lane_observer is not None and lane_abs_C is not None
            and args.lane_init_from_obs and not lane_inited and stamps):
        # 静止起始段扫描：从离起步最近的帧往回看，找"前进方向族"连续观测。
        # parking01 车停在库位内对着垂线族 → 这里通常不命中，由运动段惰性初始化。
        _sk = sorted((imu_times[0] + stamps[p.name], p)
                     for p in sorted(Path(args.left_dir).glob("frame_*.png"))
                     if p.name in stamps and imu_times[0] + stamps[p.name] < t_motion)
        _gs = []
        for _tt, _p in _sk[-200:][::-1]:
            _im = cv2.imread(str(_p))
            if _im is None:
                continue
            _ob = lane_observer.observe(
                _im, float(np.interp(_tt, lane_imu_t, lane_roll)),
                float(np.interp(_tt, lane_imu_t, lane_pitch)))
            if not _ob["ok"] or abs(_ob["alpha_deg"]) > 45.0:
                continue
            _gs.append(math.radians(_ob["alpha_deg"]))
            if len(_gs) >= 3:
                break
        if (len(_gs) >= 3
                and max(abs(g - _gs[-1]) for g in _gs)
                < math.radians(args.lane_init_tol_deg)):
            _lane_abs_init(_gs[-1], _sk[-1][0], "静止段")
        else:
            print("  [车道航向] 静止段无'前进方向族'可用观测（库位内所见多为垂线族），"
                  "等车辆驶入通道后由运动段观测初始化")

    for i in range(n_frames):
        current_time = image_times[i]
        dt = current_time - prev_time
        wheel_distance_interval = integrate_wheel_distance(
            prev_time, current_time, odo_times, wheel_speed
        )
        
        # Process only the left image. Monocular translation scale comes from
        # forward wheel speed; the right camera is not loaded or matched.
        success, forward, lateral, yaw_delta, inliers = vo.process(
            str(left_files[i]),
            max(float(dt), 0.0),
            float(wheel_speed_interp[i]),
            wheel_distance_interval,
        )
        vo_motion_results.append(
            (
                current_time,
                int(success),
                vo.last_match_count,
                vo.last_pose_inliers,
                forward,
                lateral,
                wheel_distance_interval,
                vo.last_raw_forward,
                vo.last_raw_lateral,
                vo.last_scale_factor,
            )
        )
        
        # EKF预测：偏航必须由 IMU 率子步积分。
        # 只在图像率(3.33Hz)上取一个陀螺样例会把手带噪声混叠进积分，
        # 实测使 17 分钟 yaw 漂移从 ~1.8° 恶化到 ~41°。
        if i > 0:
            n_sub = max(1, int(round(dt * args.imu_prop_hz)))
            sub_t = prev_time + np.linspace(0.0, dt, n_sub + 1)
            sub_gz = np.interp(sub_t, imu_times, gyro_z)
            sub_ws = np.interp(sub_t, odo_times, wheel_speed)
            for k in range(n_sub):
                ekf.predict(
                    sub_t[k + 1] - sub_t[k],
                    0.5 * (sub_gz[k] + sub_gz[k + 1]),   # 梯形（中点）角速度
                    sub_ws[k],
                )
            ekf.update_wheel_speed(wheel_speed_interp[i])
        
        # EKF更新（视觉VO：仅相对位置增量，偏航交给IMU）
        if success and i > 0:
            vo_success_count += 1
            # 诊断对照（--vo_probe）：破坏图像信息但保持更新的结构与频次，
            # 用于判断精度增益究竟来自视觉信息，还是仅来自多一次协方差整形。
            if args.vo_probe == 'no_lateral':
                lateral = 0.0          # 完全不含图像信息：测量退化为纯航位推算
            elif args.vo_probe == 'flip_lateral':
                lateral = -lateral     # 幅值/统计不变，方向信息被破坏
            elif args.vo_probe == 'no_forward':
                forward = 0.0          # 只留横向（唯一真正由图像决定的分量）
            yaw = ekf.x[2]  # IMU-driven heading (reliable)
            c, s = math.cos(yaw), math.sin(yaw)
            vo_accum[0] += c * forward - s * lateral
            vo_accum[1] += s * forward + c * lateral

            meas = np.array([anchor[0] + vo_accum[0], anchor[1] + vo_accum[1]])
            if ekf.update_visual_position(meas):
                anchor = np.array([ekf.x[0], ekf.x[1]])
                vo_accum = np.zeros(2)
            vo_positions.append(meas.copy())

        # —— 车道线航向观测（P0-2，独立于 IMU/轮速的第三偏航信息源）——
        _init_probe = (lane_observer is not None and lane_abs_C is not None
                       and args.lane_init_from_obs and not lane_inited
                       and current_time < image_times[0] + 6.0)
        if lane_observer is not None and (
                wheel_speed_interp[i] > args.lane_min_speed or _init_probe):
            gz_now = float(np.interp(current_time, imu_times, gyro_z))
            if abs(np.degrees(gz_now)) < args.lane_max_gyr:      # 转弯中车辆不平行于车道
                _img = cv2.imread(str(left_files[i]))
                if _img is not None:
                    _roll = float(np.interp(current_time, lane_imu_t, lane_roll))
                    _pitch = float(np.interp(current_time, lane_imu_t, lane_pitch))
                    ob = lane_observer.observe(_img, _roll, _pitch)
                    nis_log = np.nan
                    accepted = False
                    if ob["ok"]:
                        _a = math.radians(ob["alpha_deg"])
                        _C = lane_abs_C if lane_abs_C is not None else ekf.x[5]
                        r_deg = (min(max(ob["spread_deg"], 0.2), args.lane_gate_spread)
                                 if args.lane_adaptive_R else args.lane_noise_deg)
                        grid = ekf.wrap_mod(ekf.x[2] + _a - _C, math.pi / 2)
                        lane_buf.append(grid)
                        lane_buf = lane_buf[-8:]
                        _dev = max(abs(ekf.wrap_mod(v - lane_buf[-1], math.pi / 2))
                                   for v in lane_buf)
                        consistent = (
                            len(lane_buf) >= args.lane_min_consistent
                            and _dev < math.radians(args.lane_consistency_deg))
                        if (lane_abs_C is not None and args.lane_init_from_obs
                                and not lane_inited
                                and len(lane_buf) >= 3
                                and abs(ob["alpha_deg"]) <= 45.0
                                and _dev < math.radians(args.lane_init_tol_deg)):
                            # 绝对偏航初始化：车辆已驶入通道，α 属"前进方向族"
                            _lane_abs_init(_a, current_time, "运动段")
                            lane_buf = []      # 换系后旧缓冲作废
                        elif consistent and not args.lane_init_only:
                            accepted = ekf.update_lane_heading(_a, r_deg, lane_abs_C)
                            nis_log = ekf.lane_nis[-1]
                    lane_records.append((current_time, _roll, _pitch,
                                         ob["alpha_deg"], ob["spread_deg"], ob["frac"],
                                         ob["nseg"], int(ob["ok"]), int(accepted), nis_log))
        
        # 保存结果
        cov_peak = np.maximum(
            cov_peak, np.sqrt(np.maximum(np.diag(ekf.P)[:3], 0.0))
        )
        results.append({
            'time': current_time,
            'x': ekf.x[0],
            'y': ekf.x[1],
            'yaw': ekf.x[2],
            'speed': ekf.x[3]
        })
        
        prev_time = current_time
        
        if (i + 1) % 100 == 0:
            print(f"  处理进度: {i + 1}/{n_frames}")
    
    print(f"\n融合完成！")
    
    # NIS statistics
    accepted_visual = ekf.total_updates - ekf.outliers
    print(f"\nNIS卡方检验统计:")
    print(f"  VO成功帧: {vo_success_count}/{n_frames}")
    print(f"  视觉更新: {accepted_visual}/{ekf.total_updates} 通过门控")
    print(f"  门控拒绝: {ekf.outliers} 次 (卡方95%临界值: {ekf.CHI2_95[2]:.3f})")
    if ekf.visual_nis:
        v = np.array(ekf.visual_nis)
        print(f"  视觉 NIS: 均值 {v.mean():.3f}, 中位数 {np.median(v):.3f}, "
              f"范围 {v.min():.3f} ~ {v.max():.3f}  (95%门限 {ekf.CHI2_95[2]:.3f})")
    if ekf.wheel_nis:
        w = np.array(ekf.wheel_nis)
        print(f"  轮速 NIS: 均值 {w.mean():.4f}, 最大 {w.max():.4f}, "
              f"拒绝 {ekf.wheel_gate_rejects} 次  (一个子步速度量级，非独立观测量)")
    if ekf.lane_heading and ekf.lane_nis:
        v = np.array(ekf.lane_nis)
        print(f"  车道航向 NIS: 均值 {v.mean():.3f}, 中位数 {np.median(v):.3f}, "
              f"范围 {v.min():.3f} ~ {v.max():.3f}  (95%门限 {ekf.CHI2_95[1]:.3f})")
        print(f"  车道航向更新: {ekf.lane_updates} 次通过 / {len(v)} 次尝试, "
              f"NIS 拒绝 {ekf.lane_rejects} 次")
    # 协方差自洽性：滤波器自己报告的不确定度 vs 实测误差
    pstd = np.sqrt(np.maximum(np.diag(ekf.P), 0.0))
    print(f"  末端协方差 1σ: pos_x {pstd[0]:.3f} m, pos_y {pstd[1]:.3f} m, "
          f"yaw {np.degrees(pstd[2]):.2f}°, speed {pstd[3]:.3f} m/s, "
          f"gz_bias {np.degrees(pstd[4]):.5f}°/s")
    print(f"  全程最大不确定度 1σ: pos_x {cov_peak[0]:.3f} m, pos_y {cov_peak[1]:.3f} m, "
          f"yaw {np.degrees(cov_peak[2]):.2f}°")
    if ekf.lane_heading:
        print(f"  C_lane: {np.degrees(ekf.x[5]):+.3f}° "
              f"(1σ {np.degrees(math.sqrt(max(ekf.P[5, 5], 0.0))):.3f}°)"
              + (f"  [绝对模式: 固定为标定值 {np.degrees(lane_abs_C):+.3f}°]"
                 if lane_abs_C is not None else ""))
    
    # 保存结果
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 保存为CSV
    results_array = np.array([[r['time'], r['x'], r['y'], r['yaw'], r['speed']] for r in results])
    np.savetxt(
        output_dir / "fusion_results.csv",
        results_array,
        delimiter=',',
        header='time,x,y,yaw,speed',
        comments=''
    )
    np.savetxt(
        output_dir / "vo_motion_results.csv",
        np.asarray(vo_motion_results, dtype=np.float64),
        delimiter=",",
        header=(
            "time,success,matches,pose_inliers,forward_m,lateral_m,"
            "wheel_distance_m,raw_forward_m,raw_lateral_m,depth_scale_factor"
        ),
        comments="",
    )
    
    print(f"结果已保存到: {output_dir / 'fusion_results.csv'}")
    if lane_observer is not None and lane_records:
        _rec = np.asarray(lane_records, dtype=float)
        np.savetxt(
            output_dir / "lane_obs_results.csv", _rec, delimiter=",",
            header=("time,roll_online_deg,pitch_online_deg,alpha_deg,spread_deg,"
                    "frac,nseg,gate_ok,accepted,nis"),
            comments="",
        )
        print(f"  车道观测: 尝试 {len(_rec)} 帧, 门限内 {int(_rec[:, 7].sum())} 帧, "
              f"被 EKF 采纳 {int(_rec[:, 8].sum())} 帧 → lane_obs_results.csv")
    if depth_filter is not None:
        rejected_ratio = (
            depth_filter.matches_rejected / depth_filter.matches_seen
            if depth_filter.matches_seen
            else 0.0
        )
        stats_path = output_dir / "depth_filter_stats.txt"
        stats_path.write_text(
            "Depth Anything V2 metric Hypersim vits VO diagnostics\n"
            f"checkpoint: {depth_filter.checkpoint_path}\n"
            f"translation_source: {args.vo_translation_source}\n"
            f"input_size: {args.depth_input_size}\n"
            f"edge_threshold: {args.depth_edge_threshold}\n"
            f"edge_radius_px: {args.depth_edge_radius}\n"
            f"scale_samples: {dynamic_scale.samples_seen if dynamic_scale else 0}\n"
            f"scale_samples_rejected: {dynamic_scale.samples_rejected if dynamic_scale else 0}\n"
            f"scale_calibration_samples: {len(dynamic_scale.ratios) if dynamic_scale else 0}\n"
            f"final_scale_factor: {dynamic_scale.scale if dynamic_scale else 1.0:.6f}\n"
            f"matches_seen: {depth_filter.matches_seen}\n"
            f"matches_rejected: {depth_filter.matches_rejected}\n"
            f"rejected_fraction: {rejected_ratio:.6f}\n"
        )
        print(
            f"深度边缘筛选: {depth_filter.matches_rejected}/"
            f"{depth_filter.matches_seen} 个候选匹配被移除 "
            f"({rejected_ratio:.1%}); 统计保存到 {stats_path}"
        )
    if dynamic_scale is not None:
        print(
            f"动态深度尺度: factor={dynamic_scale.scale:.4f}, "
            f"有效样本={len(dynamic_scale.ratios)}, "
            f"拒绝样本={dynamic_scale.samples_rejected}"
        )
    
    # 绘制轨迹
    plot_trajectory(results, vo_positions, output_dir)
    
    return results


def plot_trajectory(results: List[dict], vo_positions: List[np.ndarray], output_dir: Path):
    """绘制轨迹图"""
    
    times = [r['time'] for r in results]
    x_est = [r['x'] for r in results]
    y_est = [r['y'] for r in results]
    
    if vo_positions:
        vo_positions = np.array(vo_positions)
    
    # 创建图形
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 1. 估计轨迹
    ax = axes[0, 0]
    ax.plot(x_est, y_est, 'b-', linewidth=1.5, label='EKF估计轨迹')
    if vo_positions is not None and len(vo_positions) > 0:
        ax.plot(vo_positions[:, 0], vo_positions[:, 1], 'r--', linewidth=1, label='VO轨迹', alpha=0.7)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('估计轨迹')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # 2. X位置
    ax = axes[0, 1]
    ax.plot(times, x_est, 'b-', linewidth=1.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('X (m)')
    ax.set_title('X位置')
    ax.grid(True, alpha=0.3)
    
    # 3. Y位置
    ax = axes[1, 0]
    ax.plot(times, y_est, 'b-', linewidth=1.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Y位置')
    ax.grid(True, alpha=0.3)
    
    # 4. 偏航角
    ax = axes[1, 1]
    yaw = [r['yaw'] for r in results]
    ax.plot(times, yaw, 'b-', linewidth=1.5)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('偏航角 (rad)')
    ax.set_title('偏航角')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'trajectory.png', dpi=150)
    print(f"轨迹图已保存到: {output_dir / 'trajectory.png'}")
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='左目单目视觉+IMU+轮速计融合定位')
    
    parser.add_argument('--calibration', type=str, 
                        default='calibration.yaml',
                        help='相机标定文件路径')
    parser.add_argument('--left_dir', type=str,
                        default='stereo_images/left',
                        help='左目图像目录（唯一使用的图像输入）')
    parser.add_argument('--imu_file', type=str,
                        default='../parking01/parking01_ADIS16465_IMU.txt',
                        help='IMU数据文件')
    parser.add_argument('--odo_file', type=str,
                        default='../parking01/parking01_RANGER_ODO.txt',
                        help='轮速计数据文件')
    parser.add_argument('--output_dir', type=str,
                        default='plots/stereo_fusion',
                        help='输出目录')
    
    parser.add_argument('--image_fps', type=float, default=3.3333,
                        help='图像帧率（仅在缺少 timestamps.txt 时用于合成时间戳）')
    parser.add_argument('--timestamps_file', type=str, default=None,
                        help='每帧真实采集时间戳（默认 <left_dir>/timestamps.txt）')
    parser.add_argument('--image_scale', type=float, default=0.5,
                        help='图像处理缩放比例')
    parser.add_argument('--max_frames', type=int, default=100000,
                        help='最大处理帧数（默认不限，跑完整轨迹）')
    parser.add_argument('--imu_prop_hz', type=float, default=50.0,
                        help='EKF预测子步频率（Hz）。必须远高于图像率，否则陀螺噪声'
                             '混叠进偏航积分：3.33Hz时17分钟漂移41°，50Hz时仅1.8°')
    
    parser.add_argument('--wheel_speed_noise', type=float, default=0.15,
                        help='轮速计噪声')
    parser.add_argument('--wheel_track_width', type=float, default=0.364,
                        help='左右轮等效轮距（米），AgileX Ranger MINI3实测364mm')
    parser.add_argument('--visual_position_noise', type=float, default=3.5,
                        help='视觉位置噪声（实测最优≈3.5 m）')
    parser.add_argument('--depth_checkpoint', type=str, default=None,
                        help='可选：Depth Anything V2 metric-depth checkpoint（默认使用项目内 Hypersim Vits 权重）')
    parser.add_argument('--vo_translation_source', type=str, default='wheel',
                        choices=['wheel', 'depth', 'depth_dynamic'],
                        help='视觉平移尺度来源：wheel=本质矩阵方向+轮速尺度；'
                             'depth=DA米制深度反投影+RANSAC PnP；'
                             'depth_dynamic=DA平移经轮速在线动态尺度校正')
    parser.add_argument('--depth_input_size', type=int, default=518,
                        help='Depth Anything 推理输入尺寸')
    parser.add_argument('--depth_edge_threshold', type=float, default=0.08,
                        help='log-depth 梯度边缘阈值；越低筛除越多匹配')
    parser.add_argument('--depth_edge_radius', type=int, default=2,
                        help='深度边缘膨胀半径（处理图像像素）')
    parser.add_argument('--depth_scale_window', type=int, default=11,
                        help='动态尺度校正的滑动中值窗口大小')
    parser.add_argument('--depth_scale_min_samples', type=int, default=5,
                        help='动态尺度开始应用前的最少有效校准样本')
    parser.add_argument('--depth_scale_min_wheel_distance', type=float, default=0.05,
                        help='更新动态尺度所需最小轮速位移（米）')
    parser.add_argument('--depth_scale_min_depth_distance', type=float, default=0.03,
                        help='更新动态尺度所需最小DA位移（米）')
    parser.add_argument('--depth_scale_min_inliers', type=int, default=20,
                        help='更新动态尺度所需最少PnP内点数')
    parser.add_argument('--depth_scale_min_factor', type=float, default=0.05,
                        help='允许的最小动态尺度因子')
    parser.add_argument('--depth_scale_max_factor', type=float, default=20.0,
                        help='允许的最大动态尺度因子')
    parser.add_argument('--visual_update_yaw', action='store_true',
                        help='允许视觉位置更新经协方差耦合修正 yaw（默认关闭：'
                             '单目VO平移方向用本EKF的yaw旋回世界系，'
                             '与预测冗余，放开会污染偏航）')
    parser.add_argument('--vo_probe', type=str, default='normal',
                        choices=['normal', 'no_lateral', 'flip_lateral', 'no_forward'],
                        help='诊断对照：破坏VO的图像信息但保留更新结构与频次，'
                             '用于分离"视觉信息增益"与"协方差整形增益"')
    parser.add_argument('--vo_scale_integral', action='store_true',
                        help='单目平移用轮速梯形积分距离定标（与EKF预测位移一致），'
                             '而非瞬时轮速×Δt；后者在加减速段带来约6%尺度偏差')
    parser.add_argument('--init_method', type=str, default='imu_alignment',
                        choices=['imu_alignment', 'groundtruth'],
                        help='初始化方式：imu_alignment=部署模式(无真值, yaw=0里程计系, '
                             '自动跳过静止段, 静止对准零偏); '
                             'groundtruth=对照模式(从真值取初始yaw与固定skip)')
    parser.add_argument('--groundtruth', type=str,
                        default='../parking01/parking01_groundtruth.nav',
                        help='真值文件（仅 --init_method groundtruth 时使用）')
    parser.add_argument('--apply_gyro_bias', action='store_true',
                        help='应用IMU静止对准估计的陀螺零偏修正（默认仅报告；'
                             '本数据集常值零偏修正无益）')
    # ===== P0-2 车道线航向观测通道（独立偏航源，默认关闭） =====
    parser.add_argument('--lane_heading', action='store_true',
                        help='启用车道线航向观测（Step 0 验证：直行帧一致性残差 '
                             'P68 0.38°；实现见 lane_heading.py / lane_probe.py）')
    parser.add_argument('--lane_tau', type=float, default=10.0,
                        help='在线姿态互补滤波时间常数（秒）')
    parser.add_argument('--lane_min_speed', type=float, default=0.05,
                        help='轮速高于此值才观测车道线（m/s）。α 与自车姿态无关，'
                             '起步瞬间 ψ=C0−α 即成立，故默认取很小值，让绝对初始化'
                             '尽早发生（起点位移≈0，避免先积分错航向再补位置）')
    parser.add_argument('--lane_init_only', action='store_true',
                        help='只允许绝对初始化换系、不跑后续车道更新（用于分离'
                             '“初始化代价”与“持续更新代价”的诊断实验）')
    parser.add_argument('--yaw_q', type=float, default=0.003,
                        help='偏航过程噪声 Q[2,2]（rad²/s）。默认 0.003=归档基线；'
                             '1e-6≈0.06°/√s 随机游走量级（P1-1 协方差病态修复实验）')
    parser.add_argument('--lane_max_gyr', type=float, default=6.0,
                        help='|陀螺z|超过该值(°/s)不观测：转弯中车辆不平行于车道')
    parser.add_argument('--lane_gate_spread', type=float, default=0.60,
                        help='自参照门限：α 众数内散布上限（°）。Step 0 实测该门限'
                             '把 >3° 离群从 4.4% 压到 0%')
    parser.add_argument('--lane_noise_deg', type=float, default=0.40,
                        help='固定观测标准差（°），Step 0 在线姿态下 P68≈0.38°')
    parser.add_argument('--lane_adaptive_R', action='store_true',
                        help='自适应 R：按每帧众数内散布 clip 到 [0.2, gate]°')
    parser.add_argument('--lane_min_consistent', type=int, default=5,
                        help='mod-90 网格一致性缓冲：需连续多少个门限内有效观测')
    parser.add_argument('--lane_consistency_deg', type=float, default=3.0,
                        help='缓冲内 ψ+α−C 允许的最大离散（°）')
    parser.add_argument('--lane_init_tol_deg', type=float, default=8.0,
                        help='绝对初始化专用的放宽互差阈值（°）：≥3 个门限内观测'
                             '互差小于该值即可初始化航向（位移尚小时完成换系）')
    parser.add_argument('--lane_abs_cal_deg', type=float, default=None,
                        help='设施级一次性标定常数 C0（°）→ 绝对航向模式；'
                             '不给则为自洽模式（C 为第 6 状态，免任何绝对信息）')
    parser.add_argument('--lane_quadrant', type=int, default=0, choices=[0, 1, 2, 3],
                        help='绝对模式初始象限 k（ψ0 = C0 − α + k·90°），'
                             '等效罗盘粗方向，一次性设置输入')
    parser.add_argument('--lane_init_from_obs', action='store_true',
                        help='绝对模式：由首个满足一致性缓冲的车道观测设定初始绝对航向')
    
    args = parser.parse_args()
    if args.lane_abs_cal_deg is not None and not args.lane_heading:
        parser.error('--lane_abs_cal_deg 需要配合 --lane_heading 使用')
    
    print("=" * 60)
    print("左目单目视觉+IMU+轮速计融合定位系统")
    print("数据集: i2Nav-Robot parking01")
    print("=" * 60)
    
    run_fusion(args)


if __name__ == '__main__':
    main()
