#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lane_heading.py —— P0-2 部署模块：在线俯仰跟踪 + 车道线航向观测

Step 0 探针（lane_probe.py）已证明：车道线方向角 α 与真值航向的一致性残差
标准差 0.20°（P68 0.17°）。但那条结果用的是**真值时变姿态**；探针的灵敏度表
同时证明精度由姿态知识决定（roll 无关，pitch 要紧，pitch 在行程中变化 ±0.5°）。
本模块提供部署可用的两块拼图：

1. ComplementaryAttitude —— 陀螺传播 + 加速度计低频修正的 roll/pitch 互补滤波
   （FRD 机体系，与 stationary_alignment 同一套角度定义，只用过去数据，因果可部署）。
   机动污染用两条物理门限抑制：
     * pitch 修正门：纵向加速度（轮速微分）|v̇| < 0.4 m/s²（1 m/s² ≈ 5.8° 俯仰假倾斜）
     * roll  修正门：向心加速度 |v·gz| < 0.4 m/s²（转弯 1.3 m/s @ 0.3 rad/s ≈ 2.3°）
   校验入口：python lane_heading.py --validate（与真值比对仅用于**评估**，不是输入）

2. LaneHeadingObserver —— 把 lane_probe.py 的几何（去畸变、地面法线、Kᵀ·L、
   地距门限、加倍角环形众数）封装成逐帧观测器；α 用**在线姿态**计算。
   输出按 Step 0 验证过的自参照门限判有效：众数权重占比 ≥0.80 且段内散布 ≤0.60°。

观测模型（Step 0 实测关系）：ψ + α ≡ C (mod 90°)，其中
  ψ  = 载体偏航，α = 车道族方向相对相机前向的夹角，
  C  = 设施常数（车道网方位 + 相机安装偏置 + 起点-车道夹角基准，场地级一次性标定，
       parking01 实测 −1.568°，时段间漂移 ≤0.29°）。
"""

import argparse
import math
import os

import cv2
import numpy as np

import lane_probe as lp

HERE = os.path.dirname(os.path.abspath(__file__))

G2 = 9.80665


def wrap_mod(a_deg, period_deg):
    """把无向/网格角折回 [-period/2, period/2)。"""
    p = float(period_deg)
    return (np.asarray(a_deg, float) + p / 2.0) % p - p / 2.0


# ----------------------------------------------------------------------------
# 1. 在线姿态（roll/pitch）互补滤波
# ----------------------------------------------------------------------------
class ComplementaryAttitude:
    """FRD 机体系 roll/pitch 互补滤波（因果，可部署）。

    角度定义与 stationary_alignment 完全一致：
        down = -a ;  roll = atan2(down_y, down_z) ;  pitch = atan2(-down_x, |down_horiz|)
    传播用 FRD 欧拉运动学；修正项以时间常数 tau 低通逼近加速度计解算值。
    """

    def __init__(self, roll0_deg=0.0, pitch0_deg=0.0, tau=10.0):
        self.roll = float(roll0_deg)
        self.pitch = float(pitch0_deg)
        self.tau = float(tau)

    def update(self, dt, gyro, accel, lon_accel=0.0, speed=0.0,
               acc_gate=1.0, lon_gate=0.4, lat_gate=0.4):
        """推进一步并返回 (roll_deg, pitch_deg)。

        gyro: (3,) rad/s FRD (gx,gy,gz)   accel: (3,) m/s²
        lon_accel: 轮速微分（前向加速度估计, m/s²）  speed: m/s
        """
        gx, gy, gz = (float(g) for g in gyro)
        r, p = math.radians(self.roll), math.radians(self.pitch)
        # —— 陀螺传播（FRD 标准欧拉运动学，roll/pitch 很小时交叉项本就微小，保留完整式）
        dr = gx + (gy * math.sin(r) + gz * math.cos(r)) * math.tan(p)
        dp = gy * math.cos(r) - gz * math.sin(r)
        self.roll += math.degrees(dr) * dt
        self.pitch += math.degrees(dp) * dt
        # —— 加速度计解算（与 stationary_alignment 同一公式）
        down = -np.asarray(accel, dtype=float)
        dn = np.linalg.norm(down)
        if dn < 1e-6:
            return self.roll, self.pitch
        roll_a = math.degrees(math.atan2(down[1], down[2]))
        pitch_a = math.degrees(math.atan2(-down[0], math.hypot(down[1], down[2])))
        # —— 污染门限 + 互补修正
        k = dt / (dt + self.tau)
        if abs(dn - G2) < acc_gate:
            if abs(speed * gz) < lat_gate:        # 向心加速度污染 roll 解算
                self.roll += k * wrap_mod(roll_a - self.roll, 360.0)
            if abs(lon_accel) < lon_gate:         # 纵向加减速污染 pitch 解算
                self.pitch += k * wrap_mod(pitch_a - self.pitch, 360.0)
        return self.roll, self.pitch


def run_attitude_track(imu_times, gyro, accel, roll0_deg, pitch0_deg,
                       tau=10.0, wheel_times=None, wheel_speed=None):
    """整段因果积分 roll/pitch。lon_accel 由轮速序列中心差分给出。"""
    n = len(imu_times)
    lon = np.zeros(n)
    if wheel_times is not None and wheel_speed is not None and len(wheel_speed) > 2:
        w = np.interp(imu_times, wheel_times, wheel_speed)
        d = np.gradient(imu_times)
        lon = np.gradient(w) / d
    roll_arr = np.zeros(n)
    pitch_arr = np.zeros(n)
    f = ComplementaryAttitude(roll0_deg, pitch0_deg, tau)
    speed_arr = np.interp(imu_times, wheel_times, wheel_speed) \
        if wheel_times is not None and wheel_speed is not None else np.zeros(n)
    for i in range(n):
        dt = (imu_times[i] - imu_times[i - 1]) if i else 0.005
        if dt <= 0:
            dt = 0.005
        r, p = f.update(dt, gyro[i], accel[i], lon_accel=float(lon[i]),
                        speed=float(speed_arr[i]))
        roll_arr[i], pitch_arr[i] = r, p
    return roll_arr, pitch_arr


# ----------------------------------------------------------------------------
# 2. 车道航向观测器
# ----------------------------------------------------------------------------
class LaneHeadingObserver:
    """逐帧车道线方向角观测器（几何与门限与 lane_probe Step 0 完全同口径）。"""

    CFG = dict(roi_below_h=8, roi_bottom=1200, canny1=60, canny2=160,
               hough_thr=50, min_len=110, max_gap=10,
               max_abs_alpha=80.0, r_min=1.0, r_max=16.0)
    GATE_FRAC = 0.80       # Step 0：该门限把 >3° 离群从 4.4% 压到 0%，保留 51%
    GATE_SPREAD = 0.60
    SPREAD_FLOOR = 0.20    # R 自适应下限（°）

    def __init__(self, calibration_file, cfg=None):
        cal = lp.load_calibration(calibration_file)
        self.K, self.dist = cal["K"], cal["dist"]
        self.R_bc = cal["R_bc"]
        self.height = float(abs(cal["odo_lever"][2]) + abs(cal["t_bc"][2]))
        size = (cal["res"][0], cal["res"][1])
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.K, None, None, self.K, size, cv2.CV_32FC1)
        self.cfg = dict(self.CFG)
        if cfg:
            self.cfg.update(cfg)

    def observe(self, img_bgr, roll_deg, pitch_deg):
        """返回 dict(ok, alpha_deg, spread_deg, frac, nseg)；alpha 为 (-90,90] 有符号。"""
        und = cv2.remap(img_bgr, self.map1, self.map2, cv2.INTER_LINEAR)
        R_wb = lp.euler_321_to_dcm(roll_deg, pitch_deg, 0.0)   # 结果与 yaw 无关
        up_c = lp.up_in_camera(R_wb, self.R_bc)
        segs, _lo, _hi, _vh = lp.extract(und, self.K, up_c, self.height, self.cfg)
        if not segs:
            return dict(ok=False, alpha_deg=np.nan, spread_deg=np.nan,
                        frac=0.0, nseg=0)
        al = np.array([s[0] for s in segs])
        w = np.array([s[1] for s in segs])
        modes = lp._circ_modes(al, w, wrap=180.0, bin_deg=1.0, win_deg=3.0)
        if not modes:
            return dict(ok=False, alpha_deg=np.nan, spread_deg=np.nan,
                        frac=0.0, nseg=len(segs))
        m1 = modes[0]
        ok = (m1["frac"] >= self.GATE_FRAC) and (m1["spread"] <= self.GATE_SPREAD)
        return dict(ok=bool(ok), alpha_deg=float(lp.wrap_sym(m1["center"])),
                    spread_deg=float(m1["spread"]), frac=float(m1["frac"]),
                    nseg=len(segs))


# ----------------------------------------------------------------------------
# 校验入口：在线姿态 vs 真值（真值仅用于评估）
# ----------------------------------------------------------------------------
def _validate(args):
    import stereo_imu_odo_fusion_v2 as core

    t, gyro, accel = core.load_imu_full(args.imu_file)
    odo_t, odo_v = core.load_wheel_speed_data(args.odo_file, 0.364)
    align = core.stationary_alignment(args.imu_file)
    roll_arr, pitch_arr = run_attitude_track(
        t, gyro, accel, align["roll_deg"], align["pitch_deg"],
        tau=args.tau, wheel_times=odo_t, wheel_speed=odo_v)

    g = np.loadtxt(args.groundtruth)
    gt_t, gt_roll, gt_pitch = g[:, 0], g[:, 7], g[:, 8]
    m = (t >= gt_t[0]) & (t <= gt_t[-1]) & (t >= align["t_motion_start"])
    gr = np.interp(t[m], gt_t, gt_roll)
    gp = np.interp(t[m], gt_t, gt_pitch)
    dr = wrap_mod(roll_arr[m] - gr, 360.0)
    dp = wrap_mod(pitch_arr[m] - gp, 360.0)

    lines = []
    lines.append("在线姿态互补滤波 vs 真值（真值仅用于评估，非输入）  n=%d 样本(200Hz)" % m.sum())
    for tag, d in (("roll ", dr), ("pitch", dp)):
        lines.append("  %s: 均值 %+.3f°  RMSE %.3f°  P95 |e| %.3f°  最大 |e| %.3f°"
                     % (tag, d.mean(), float(np.sqrt((d ** 2).mean())),
                        float(np.percentile(np.abs(d), 95)), float(np.abs(d).max())))
    lines.append("  GT 姿态行程内变化幅度: roll ±%.2f°  pitch ±%.2f°（相对起始对准值）"
                 % (np.abs(gr - gr[0]).max(), np.abs(gp - gp[0]).max()))
    lines.append("  参考：Step 0 探针若用固定起始姿态，航向一致性残差 P95=2.92°；"
                 "若用真值时变姿态，std=0.20°")
    txt = "\n".join(lines)
    print(txt)
    out = os.path.join(HERE, "plots", "lane_probe", "attitude_track.txt")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, "w", encoding="utf-8").write(txt + "\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = [
            "SimHei", "Noto Sans CJK SC", "Noto Sans CJK JP",
            "WenQuanYi Zen Hei", "AR PL UMing CN", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        fig, ax = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
        ts = t[m] - t[m][0]
        ax[0].plot(ts, gp, lw=.8, label="GT pitch")
        ax[0].plot(ts, pitch_arr[m], lw=.8, label="在线 pitch (tau=%.0fs)" % args.tau)
        ax[0].legend(); ax[0].set_ylabel("pitch (°)")
        ax[0].set_title("在线姿态跟踪 vs 真值（RMSE %.3f°）" %
                        float(np.sqrt((dp ** 2).mean())))
        ax[1].plot(ts, gr - gr[0], lw=.8, label="GT roll 变化")
        ax[1].plot(ts, roll_arr[m] - align["roll_deg"], lw=.8, label="在线 roll 变化")
        ax[1].axhline(0, lw=.5, color="k")
        ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("相对起始的 roll 变化 (°)")
        ax[1].legend()
        fig.tight_layout()
        fig.savefig(os.path.join(HERE, "plots", "lane_probe", "attitude_track.png"), dpi=110)
        print("-> plots/lane_probe/attitude_track.png / .txt")
    except Exception as e:
        print("出图跳过:", e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--imu-file", default=os.path.join(HERE, "..", "parking01",
                                                       "parking01_ADIS16465_IMU.txt"))
    ap.add_argument("--odo-file", default=os.path.join(HERE, "..", "parking01",
                                                       "parking01_RANGER_ODO.txt"))
    ap.add_argument("--groundtruth", default=os.path.join(HERE, "..", "parking01",
                                                          "parking01_groundtruth.nav"))
    ap.add_argument("--tau", type=float, default=10.0)
    args = ap.parse_args()
    if args.validate:
        _validate(args)


if __name__ == "__main__":
    main()
