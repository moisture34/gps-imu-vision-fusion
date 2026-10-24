#!/usr/bin/env python3
"""
Indoor GNSS-denied monocular/IMU/wheel-odometry fusion validation.

This is an executable research baseline built around the supplied parking01 data:
  * monocular PNGs (left camera only) provide frame-to-frame visual odometry;
  * the ADIS/MID360 IMU provides yaw-rate prediction;
  * RANGER wheel odometry provides forward-speed updates;
  * parking01_groundtruth.nav is used only for offline evaluation.

The image dump has no timestamps or calibration sidecars.  The defaults therefore
use IMAGE_FPS=10 Hz, a pinhole focal-length estimate.
Replace those values with the real camera/clock calibration before reporting metric
results in a paper.

注意：单目视觉里程计使用本质矩阵分解来估计相机运动，无法直接获取绝对尺度。
尺度通过轮速计和IMU数据进行恢复。
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from wheel_odometry import compute_wheel_odometry


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def load_ascii(path: Path) -> np.ndarray:
    return np.loadtxt(path, dtype=np.float64)


def quat_to_yaw_deg(qx: np.ndarray, qy: np.ndarray, qz: np.ndarray, qw: np.ndarray) -> np.ndarray:
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return np.unwrap(yaw)


def interp(values: np.ndarray, src_t: np.ndarray, dst_t: np.ndarray) -> np.ndarray:
    return np.interp(dst_t, src_t, values)


def load_reference(root: Path, image_times: np.ndarray) -> dict[str, np.ndarray]:
    gt = load_ascii(root / "parking01_groundtruth.nav")
    trajectory = load_ascii(root / "parking01_trajectory.csv")
    # groundtruth.nav: t, x, y, z, vx, vy, vz, roll_deg, pitch_deg, yaw_deg
    yaw_gt = np.unwrap(np.deg2rad(gt[:, 9]))
    return {
        "x": interp(gt[:, 1], gt[:, 0], image_times),
        "y": interp(gt[:, 2], gt[:, 0], image_times),
        "yaw": interp(yaw_gt, gt[:, 0], image_times),
        "vx": interp(gt[:, 4], gt[:, 0], image_times),
        "vy": interp(gt[:, 5], gt[:, 0], image_times),
        "trajectory_yaw": interp(
            quat_to_yaw_deg(
                trajectory[:, 4], trajectory[:, 5], trajectory[:, 6], trajectory[:, 7]
            ),
            trajectory[:, 0],
            image_times,
        ),
    }


def load_imu(root: Path) -> tuple[np.ndarray, np.ndarray]:
    # The supplied IMU files are t, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z.
    data = load_ascii(root / "parking01_ADIS16465_IMU.txt")
    return data[:, 0], data[:, 3]


def load_wheel_speed(
    root: Path, track_width_m: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    # RANGER first four channels are the four wheel speed channels in this dump.
    data = load_ascii(root / "parking01_RANGER_ODO.txt")
    speed, yaw_rate = compute_wheel_odometry(data[:, 1:5], track_width_m)
    return data[:, 0], speed, yaw_rate


def build_camera_matrix(width: int, height: int, scale: float, focal_ratio: float) -> np.ndarray:
    fx = focal_ratio * width * scale
    fy = fx
    return np.array(
        [[fx, 0.0, 0.5 * width * scale], [0.0, fy, 0.5 * height * scale], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


class MonoVisualOdometry:
    """单目视觉里程计 - 仅使用左目图像"""
    
    def __init__(self, width: int, height: int, scale: float, baseline: float, focal_ratio: float):
        try:
            import cv2
        except ImportError as exc:  # pragma: no cover - exercised by environment setup
            raise RuntimeError("opencv-python-headless is required for monocular visual odometry") from exc
        self.cv2 = cv2
        self.scale = scale
        self.K = build_camera_matrix(width, height, scale, focal_ratio)
        self.baseline = baseline  # 保留用于尺度恢复
        self.detector = cv2.ORB_create(nfeatures=1400, fastThreshold=12)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.prev_kp = None
        self.prev_desc = None
        self.prev_img = None
        self.scale_factor = 1.0  # 单目尺度因子（动态校准）
        self.prev_t = None
        self.calibration_samples = []  # 用于尺度校准的样本

    def _read_gray(self, path: Path) -> np.ndarray:
        # cv2.imread on Windows can fail for non-ASCII workspace paths. Decode
        # the bytes through NumPy so the path never crosses the narrow Win32 API.
        encoded = np.fromfile(str(path), dtype=np.uint8)
        image = self.cv2.imdecode(encoded, self.cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(path)
        if self.scale != 1.0:
            image = self.cv2.resize(image, None, fx=self.scale, fy=self.scale, interpolation=self.cv2.INTER_AREA)
        return image

    def calibrate_scale(self, wheel_speed: float, dt: float, vo_forward: float):
        """
        用轮速计校准单目尺度因子
        
        原理：轮速计提供真实速度，单目VO提供相对位移
        尺度 = 真实位移 / 视觉位移
        
        Args:
            wheel_speed: 轮速计速度 (m/s)
            dt: 时间间隔 (s)
            vo_forward: 单目VO估计的前向位移（未校准）
        """
        # 只在轮速计有效且VO有有效位移时校准
        if dt <= 0 or abs(vo_forward) < 1e-6 or abs(wheel_speed) < 0.01:
            return
        
        # 轮速计提供的真实位移
        true_displacement = wheel_speed * dt
        
        # 计算尺度因子
        scale_sample = true_displacement / vo_forward
        
        # 收集样本（前50帧用于校准，确保有足够有效数据）
        if len(self.calibration_samples) < 50:
            self.calibration_samples.append(scale_sample)
            # 使用中位数作为尺度因子（鲁棒性更好）
            if len(self.calibration_samples) >= 10:
                self.scale_factor = float(np.median(self.calibration_samples))
    
    def process(self, left_path: Path, current_t: float = None) -> tuple[bool, float, float, float, int]:
        """处理单目图像"""
        img = self._read_gray(left_path)
        kp, desc = self.detector.detectAndCompute(img, None)
        
        if self.prev_desc is None or desc is None or self.prev_kp is None:
            self.prev_kp, self.prev_desc, self.prev_img = kp, desc, img
            if current_t is not None:
                self.prev_t = current_t
            return False, 0.0, 0.0, 0.0, 0
        
        if desc is None or len(kp) < 8 or len(self.prev_kp) < 8:
            return False, 0.0, 0.0, 0.0, 0
        
        # 特征点匹配
        matches = self.matcher.knnMatch(self.prev_desc, desc, k=2)
        pts_prev = []
        pts_curr = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                pts_prev.append(self.prev_kp[m.queryIdx].pt)
                pts_curr.append(kp[m.trainIdx].pt)
        
        pts_prev = np.array(pts_prev, dtype=np.float32)
        pts_curr = np.array(pts_curr, dtype=np.float32)
        
        success = False
        forward = lateral = yaw_delta = 0.0
        inliers = 0
        
        if len(pts_prev) >= 8:
            # 计算本质矩阵
            E, mask = self.cv2.findEssentialMat(pts_prev, pts_curr, self.K, 
                                                 method=self.cv2.RANSAC, 
                                                 prob=0.999, 
                                                 threshold=1.0)
            if E is not None and mask is not None:
                inliers = int(mask.sum())
                
                if inliers >= 8:
                    # 从本质矩阵恢复运动
                    _, R, t, mask_pose = self.cv2.recoverPose(E, pts_prev, pts_curr, self.K)
                    
                    if R is not None and t is not None:
                        # 计算相机运动
                        t = t.reshape(3)
                        # 使用动态校准的尺度因子恢复真实尺度
                        t = t * self.scale_factor
                        
                        # Camera optical z is taken as robot forward; camera x is right.
                        forward = float(t[2])
                        lateral = float(t[0])
                        yaw_delta = float(math.atan2(R[0, 2], R[2, 2]))
                        success = np.isfinite([forward, lateral, yaw_delta]).all()
        
        self.prev_kp, self.prev_desc, self.prev_img = kp, desc, img
        if current_t is not None:
            self.prev_t = current_t
        return success, forward, lateral, yaw_delta, inliers


class PlanarEkf:
    """Small planar EKF: [x, y, yaw, speed, gyro_bias]."""

    def __init__(
        self,
        wheel_speed_noise: float,
        visual_position_noise: float,
        visual_yaw_noise: float,
        wheel_yaw_rate_noise: float = 0.08,
    ):
        self.x = np.zeros(5, dtype=np.float64)
        self.P = np.diag([1.0, 1.0, 0.3, 1.0, 0.03])
        self.wheel_speed_noise = wheel_speed_noise
        self.visual_position_noise = visual_position_noise
        self.visual_yaw_noise = visual_yaw_noise
        self.wheel_yaw_rate_noise = wheel_yaw_rate_noise

    def predict(
        self, dt: float, gyro_z: float, wheel_speed: float, wheel_yaw_rate: float
    ):
        px, py, yaw, speed, bias = self.x
        yaw_rate = gyro_z - bias
        self.x[0] = px + speed * math.cos(yaw) * dt
        self.x[1] = py + speed * math.sin(yaw) * dt
        self.x[2] = float(wrap_angle(yaw + yaw_rate * dt))
        self.x[3] = wheel_speed
        F = np.eye(5)
        F[0, 2] = -speed * math.sin(yaw) * dt
        F[0, 3] = math.cos(yaw) * dt
        F[1, 2] = speed * math.cos(yaw) * dt
        F[1, 3] = math.sin(yaw) * dt
        F[2, 4] = -dt
        Q = np.diag([0.01, 0.01, 0.003, self.wheel_speed_noise**2, 1e-5]) * max(dt, 1e-3)
        self.P = F @ self.P @ F.T + Q

        # Treat the differential-wheel rotation over this interval as a yaw
        # observation, fusing it with the IMU-propagated yaw above.
        H_yaw = np.array([[0.0, 0.0, 1.0, 0.0, 0.0]])
        wheel_yaw = float(wrap_angle(yaw + wheel_yaw_rate * dt))
        innovation = np.array([float(wrap_angle(wheel_yaw - self.x[2]))])
        R_yaw = np.array([[(self.wheel_yaw_rate_noise * dt) ** 2]])
        S_yaw = H_yaw @ self.P @ H_yaw.T + R_yaw
        K_yaw = self.P @ H_yaw.T @ np.linalg.inv(S_yaw)
        self.x = self.x + (K_yaw @ innovation).reshape(-1)
        self.x[2] = float(wrap_angle(self.x[2]))
        self.P = (np.eye(5) - K_yaw @ H_yaw) @ self.P

    def update_visual_pose(self, measurement: np.ndarray):
        H = np.zeros((3, 5))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        innovation = measurement - H @ self.x
        innovation[2] = float(wrap_angle(innovation[2]))
        R = np.diag([self.visual_position_noise**2, self.visual_position_noise**2, self.visual_yaw_noise**2])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[2] = float(wrap_angle(self.x[2]))
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_wheel_speed(self, wheel_speed: float):
        H = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
        innovation = np.array([wheel_speed - self.x[3]])
        S = H @ self.P @ H.T + np.array([[self.wheel_speed_noise**2]])
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ innovation).reshape(-1)
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_visual_position(self, measurement: np.ndarray):
        """只用视觉位置[x, y]更新EKF，不更新偏航"""
        H = np.zeros((2, 5))
        H[0, 0] = H[1, 1] = 1.0  # 只观测x, y
        innovation = measurement - H @ self.x
        R = np.diag([self.visual_position_noise**2, self.visual_position_noise**2])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[2] = float(wrap_angle(self.x[2]))  # 保持偏航在[-pi, pi]
        self.P = (np.eye(5) - K @ H) @ self.P


def run(args: argparse.Namespace) -> dict[str, float]:
    root = Path(args.root).resolve()
    left_files = sorted((root / args.left_dir).glob("frame_*.png"))
    if not left_files:
        raise RuntimeError(f"Left image folder must contain frame_*.png files")
    count = min(args.max_frames, len(left_files))
    left_files = left_files[:count]
    gt_full = load_ascii(root / "parking01_groundtruth.nav")
    start_t = args.image_start if args.image_start is not None else float(gt_full[0, 0])
    image_times = start_t + np.arange(count, dtype=np.float64) / args.image_fps
    valid = image_times <= float(gt_full[-1, 0])
    image_times = image_times[valid]
    left_files = left_files[: len(image_times)]
    ref = load_reference(root, image_times)
    imu_t, gyro_z = load_imu(root)
    odo_t, wheel_speed, wheel_yaw_rate = load_wheel_speed(
        root, args.wheel_track_width
    )
    gyro_z_i = interp(gyro_z, imu_t, image_times)
    wheel_i = interp(wheel_speed, odo_t, image_times)
    wheel_yaw_rate_i = interp(wheel_yaw_rate, odo_t, image_times)

    vo = MonoVisualOdometry(1600, 1200, args.image_scale, args.baseline, args.focal_ratio)
    ekf = PlanarEkf(
        args.wheel_speed_noise,
        args.visual_position_noise,
        args.visual_yaw_noise,
        args.wheel_yaw_rate_noise,
    )
    # A planar trajectory needs an initial yaw alignment.  If no external
    # heading is supplied, use the first reference yaw only as an offline
    # alignment for this validation baseline (never as a measurement update).
    initial_yaw = math.radians(args.initial_yaw_deg) if args.initial_yaw_deg is not None else float(ref["yaw"][0])
    ekf.x[2] = initial_yaw
    visual_pose = np.array([0.0, 0.0, initial_yaw], dtype=np.float64)
    records = []
    prev_t = image_times[0]
    successes = 0
    total_inliers = 0
    for i, (left, t) in enumerate(zip(left_files, image_times)):
        ok, forward, lateral, yaw_delta, inliers = vo.process(left, current_t=float(t))
        if i > 0:
            dt = max(float(t - prev_t), 1e-4)
            ekf.predict(
                dt,
                float(gyro_z_i[i]),
                float(wheel_i[i]),
                float(wheel_yaw_rate_i[i]),
            )
            ekf.update_wheel_speed(float(wheel_i[i]))
            if ok:
                # 用轮速计校准单目尺度（前50帧有效数据）
                vo.calibrate_scale(float(wheel_i[i]), dt, forward)
                
                # 偏航主要依赖IMU，不用视觉的yaw_delta
                # 因为单目VO的旋转估计不稳定
                yaw_prev = visual_pose[2]
                # 只用IMU预测偏航（在predict步骤中已完成）
                # visual_pose[2] = float(wrap_angle(yaw_prev + yaw_delta))  # 不用视觉偏航
            prev_t = float(t)
        if i > 0 and ok:
            # forward/lateral are camera-frame metric increments from VO.
            # 只用视觉更新位置，偏航由IMU预测
            c, s = math.cos(visual_pose[2]), math.sin(visual_pose[2])
            visual_pose[0] += c * forward - s * lateral
            visual_pose[1] += s * forward + c * lateral
            # 只用位置更新EKF，不用偏航
            ekf.update_visual_position(visual_pose[:2])  # 只更新[x, y]
            successes += 1
            total_inliers += inliers
        records.append([float(t), *ekf.x, *ref["x"][i : i + 1], *ref["y"][i : i + 1]])
    rec = np.asarray(records)
    out_dir = Path(args.output_dir)
    if not out_dir.is_absolute():
        out_dir = Path(__file__).resolve().parent / args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    np.savetxt(out_dir / "fusion_results.csv", rec, delimiter=",", header="time,x,y,yaw,speed,gyro_bias,gt_x,gt_y", comments="")
    ex = rec[:, 1] - rec[:, 6]
    ey = rec[:, 2] - rec[:, 7]
    errors = np.hypot(ex, ey)
    metrics = {
        "frames": float(len(rec)),
        "visual_successes": float(successes),
        "visual_success_rate": float(successes / max(1, len(rec) - 1)),
        "mean_error_m": float(np.mean(errors)),
        "median_error_m": float(np.median(errors)),
        "p95_error_m": float(np.percentile(errors, 95)),
        "max_error_m": float(np.max(errors)),
        "final_error_m": float(errors[-1]),
        "mean_visual_inliers": float(total_inliers / max(1, successes)),
    }
    with (out_dir / "metrics.txt").open("w", encoding="utf-8") as f:
        f.write("Indoor monocular + IMU + wheel odometry baseline\n")
        f.write("WARNING: image timestamps and camera calibration are approximated.\n")
        f.write("WARNING: default initial yaw is taken from the first reference pose for offline alignment.\n")
        f.write(f"image_fps={args.image_fps}\nimage_scale={args.image_scale}\nbaseline_m={args.baseline}\nfocal_ratio={args.focal_ratio}\ninitial_yaw_deg={math.degrees(initial_yaw):.6f}\n")
        f.write(f"wheel_track_width_m={args.wheel_track_width}\n")
        f.write(f"wheel_yaw_rate_noise_rad_s={args.wheel_yaw_rate_noise}\n")
        for key, value in metrics.items():
            f.write(f"{key}={value:.6f}\n")
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(ref["x"], ref["y"], "r--", lw=1.2, label="Ground truth")
    ax.plot(rec[:, 1], rec[:, 2], "b", lw=1.3, label="Monocular + IMU + wheel EKF")
    ax.set_title(f"Indoor GNSS-denied fusion (median error {metrics['median_error_m']:.3f} m)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "fusion_trajectory.png", dpi=150)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(image_times, errors, color="tab:blue")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position error (m)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "fusion_error.png", dpi=150)
    plt.close(fig)
    return metrics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".")
    parser.add_argument("--left-dir", default="images_left_full", help="左目图像目录")
    parser.add_argument("--output-dir", default="plots/indoor_fusion")
    parser.add_argument("--max-frames", type=int, default=500)
    parser.add_argument("--image-fps", type=float, default=3.56, help="图像帧率 (i2Nav-Robot parking01: 3.56Hz)")
    parser.add_argument("--image-start", type=float, default=None)
    parser.add_argument("--image-scale", type=float, default=0.5)
    parser.add_argument("--baseline", type=float, default=0.20, help="基线长度（单目模式下用于尺度恢复）")
    parser.add_argument("--focal-ratio", type=float, default=0.90)
    parser.add_argument("--initial-yaw-deg", type=float, default=None)
    parser.add_argument("--wheel-speed-noise", type=float, default=0.15)
    parser.add_argument(
        "--wheel-track-width",
        type=float,
        default=0.46,
        help="左右轮等效轮距（米），parking01数据估计值",
    )
    parser.add_argument(
        "--wheel-yaw-rate-noise",
        type=float,
        default=0.08,
        help="差速轮速角速度观测噪声标准差（rad/s）",
    )
    parser.add_argument("--visual-position-noise", type=float, default=5.0, help="视觉位置噪声（降低=更信任视觉）")
    parser.add_argument("--visual-yaw-noise", type=float, default=0.12)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    metrics = run(args)
    print("Indoor monocular + IMU + wheel EKF complete")
    for key, value in metrics.items():
        print(f"{key}: {value:.6f}")
