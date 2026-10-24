#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lane_probe.py —— P0-2 可行性探针（Step 0）

目的：在把"车道线航向约束"接进 EKF 之前，先用最便宜的经典视觉管线回答一个问题：
**地面线族的方向角 α 能不能测到 1~2°？** 若能，P0-2 成立；若不能，UFLine 权重
下下来也没用（几何提取才是瓶颈，不是分割）。

管线（不用任何深度模型）：
  1. calibration.yaml 真实内参 + radtan 去畸变（角点畸变 ±63 px ≈ 3.4°，必须去）。
  2. 重力方向 → 地面法线 → 地平线（地面消失线）。方向角 α **与相机离地高度无关**，
     只依赖 fx/fy/cx/cy + 重力 + 相机-IMU 外参（均在标定文件中）。
  3. 地面 ROI 内 Canny + 概率 Hough 提线段（颜色无关，避开反光地坪的 HSV 调参）。
  4. 严格几何：图像线 L=(a,b,c) → 观测平面法线 n = Kᵀ·L → 与地面法线叉乘得该 3D
     直线的水平方向 d → α = atan2(d·right, d·forward)。
     关键性质：**任何水平且平行于车道族的 3D 直线（地面线、墙脚线、车位边界、
     吊顶桥架）共享同一消失点，因此给出同一个 α** —— 这是本方法的鲁棒性来源。
  5. 用已知 h≈0.91 m 把线段端点反投影到地面，按真实地距 0.8~18 m 门限剔除远场杂点。
  6. α 是 mod-180 无向量 → 用"加倍角 2α"做真正的环形直方图众数（1° 精度）。
  7. 自校验（不需要真值）：主/次线族应正交，即 |α1−α2| ≈ 90°。
  8. 与真值对照：若 α 可测，则 c(t)=wrap90(α1(t)+yaw(t)) 必集中在一个窄峰内
     （停车场车道网轴对齐，两个正交族在 mod-90 下重合）。
     **c(t) 的环形标准差就是"能不能测到 1~2°"的直接答案。**

输出：plots/lane_probe/{probe_results.csv, report.txt, alpha_vs_yaw.png,
      consistency.png, seg_*.png}
"""

import argparse
import os
import time

import cv2
import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


# ----------------------------------------------------------------------------
# 标定与几何
# ----------------------------------------------------------------------------
def load_calibration(path):
    with open(path, "r", encoding="utf-8") as f:
        cal = yaml.safe_load(f)
    cam = cal["camera"]["left"]
    fx, fy, cx, cy = cam["intrinsic"]
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
    dist = np.array(cam["distortions"], dtype=float)
    T = np.array(cam["T_imu_cam"], dtype=float)      # p_imu = R @ p_cam + t
    return dict(K=K, dist=dist, R_bc=T[:3, :3], t_bc=T[:3, 3],
                odo_lever=np.array(cal["adis16465"]["odo_lever"], dtype=float),
                res=[int(v) for v in cal["camera"]["resolution"]])


def euler_321_to_dcm(roll_deg, pitch_deg, yaw_deg):
    """body -> world，z-up 航空顺序 Rz(yaw)Ry(pitch)Rx(roll)。"""
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy_, sy = np.cos(y), np.sin(y)
    return (np.array([[cy_, -sy, 0], [sy, cy_, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def up_in_camera(R_wb, R_bc):
    """世界天向表达到相机系。

    注意符号约定：calibration.yaml 的 T_imu_cam 给出 p_imu = R·p_cam + t，其列即
    相机轴在机体系中的方向，可得 cam_x≈body_y、cam_y≈body_z、cam_z≈body_x。
    相机是正立前视（cam_y 应指向图像下方=世界下方），故机体 z 轴朝**下**（NED 式
    x前-y右-z下）。而本函数的 R_wb 采用 z-up 世界系，叉乘结果符号相反，因此下面
    用物理约束强制符号：世界天向在相机系的 y 分量必须为负（天在图像上方）。
    α 本身对该符号不敏感（d 反号后被前向规范化收回，fwd/rgt 是 up 的二次式），
    但 ground_range() 反投影依赖它。
    """
    up_b = R_wb.T @ np.array([0.0, 0.0, 1.0])
    up_c = R_bc.T @ up_b
    up_c /= np.linalg.norm(up_c)
    if up_c[1] > 0:
        up_c = -up_c
    return up_c


def horizon_line(K, up_c):
    """地面消失线 l = K^-T · n_ground（l·(u,v,1)=0）。"""
    l = np.linalg.inv(K).T @ up_c
    return l / np.linalg.norm(l[:2])


def horizon_row_at(l, u):
    return None if abs(l[1]) < 1e-12 else float(-(l[0] * u + l[2]) / l[1])


def _horizontal_basis(K, up_c):
    """相机系的水平前向/右向单位向量（手性一致）。"""
    ez = np.array([0.0, 0.0, 1.0])
    ex = np.array([1.0, 0.0, 0.0])
    fwd = ez - ez.dot(up_c) * up_c
    rgt = ex - ex.dot(up_c) * up_c
    fwd /= np.linalg.norm(fwd)
    rgt /= np.linalg.norm(rgt)
    if rgt.dot(np.cross(fwd, up_c)) < 0:
        rgt = -rgt
    return fwd, rgt


def alpha_of_line(K, up_c, line, basis=None):
    """图像直线 -> 其 3D 水平方向相对相机前向的夹角 α（度，有符号，(-90,90]）。

    α>0 = 线方向偏向相机右侧。直线无向，故归到前半平面。
    """
    nv = K.T @ np.asarray(line, dtype=float)      # 观测平面法线：n = Kᵀ L
    n = np.linalg.norm(nv)
    if n < 1e-12:
        return None
    nv /= n
    d = np.cross(nv, up_c)                        # 同时在观测平面与地平面内
    nd = np.linalg.norm(d)
    if nd < 1e-9:
        return None
    d /= nd
    fwd, rgt = basis if basis is not None else _horizontal_basis(K, up_c)
    if d.dot(fwd) < 0:
        d = -d
    return float(np.degrees(np.arctan2(d.dot(rgt), d.dot(fwd))))


def ground_range(K, up_c, u, v, height):
    """像素 -> 地面点的相机系 3D 坐标（用已知离地高度 h），返回 (前向, 侧向, 距离)。"""
    ray = np.linalg.inv(K) @ np.array([u, v, 1.0])
    den = ray.dot(up_c)
    if abs(den) < 1e-9:
        return None
    lam = -height / den                    # 地面在相机下方 height 处
    if lam <= 0:
        return None
    p = lam * ray
    fwd, rgt = _horizontal_basis(K, up_c)
    y = p.dot(rgt)
    x = p.dot(fwd)
    return x, y, float(np.hypot(x, y))


# ----------------------------------------------------------------------------
# 环形统计（α 是 mod-180 无向量 -> 用加倍角 2α 做普通环形统计）
# ----------------------------------------------------------------------------
def orthogonality_check(segs, a1, tol_deg=15.0):
    """次族正交自检（免真值置信度信号）：在 α1+90°(mod 180) ± tol 邻域内约束搜集
    线段并重算族向。返回 (alpha2, spread2, frac2, orth_err)。
    理由：车库所有水平直线族（地面线/墙脚线/车位边/桥架）都贴在同一个 mod-90°
    网格上，若正交邻域内还能聚出第二族，则网格结构自洽，观测可信度更高。"""
    if not segs:
        return (np.nan, np.nan, 0.0, np.nan)
    a = np.array([s[0] for s in segs], float) % 180.0
    w = np.array([s[1] for s in segs], float)
    target = (a1 + 90.0) % 180.0
    d = np.abs(np.mod(a - target + 90.0, 180.0) - 90.0)
    sel = d <= tol_deg
    if not sel.any():
        return (np.nan, np.nan, 0.0, np.nan)
    ang = np.radians(2.0 * a[sel])
    cw = np.cos(ang) * w[sel]
    sw = np.sin(ang) * w[sel]
    mb = np.degrees(np.arctan2(sw.sum(), cw.sum())) % 360.0
    R = min(np.hypot(cw.sum(), sw.sum()) / w[sel].sum(), 1.0)
    c2 = (mb / 2.0) % 180.0
    diff = np.mod(c2 - a1, 180.0)
    return (float(c2), float(np.degrees(np.sqrt(-2.0 * np.log(max(R, 1e-9)))) / 2.0),
            float(w[sel].sum() / w.sum()), float(abs(diff - 90.0)))


def _circ_modes(a_deg, w, wrap=180.0, bin_deg=1.0, win_deg=3.0, n_max=2):
    """在 mod-wrap 圆周上找 n_max 个加权众数（加倍角 2α 做真正的环形统计）。

    返回 [{center(mod wrap), spread(度), frac(权重占比), n(点数)}, ...]
    """
    a = np.mod(np.asarray(a_deg, float), wrap)
    b = np.mod(2.0 * a, 360.0)                    # 加倍 -> 普通 360° 圆周
    b = np.asarray(b, float)
    w = np.asarray(w, float)
    step = 2.0 * bin_deg
    nb = int(round(360.0 / step))
    used = np.zeros(len(b), bool)
    out = []
    for _ in range(n_max):
        if used.all():
            break
        idx = np.minimum((b / step).astype(int), nb - 1)
        hist = np.zeros(nb)
        np.add.at(hist, idx[~used], w[~used])
        pad = np.r_[hist[-1:], hist, hist[:1]]
        hs = np.convolve(pad, np.ones(3) / 3.0, "same")[1:-1]   # 环形平滑
        if hs.sum() <= 1e-12:
            break
        peak = int(np.argmax(hs))
        pb = (peak + 0.5) * step
        dist = np.abs(np.mod(b - pb + 180.0, 360.0) - 180.0)
        sel = (dist < win_deg) & (~used)
        if sel.sum() < 3:
            break
        ang = np.radians(b[sel])
        cw = np.cos(ang) * w[sel]
        sw = np.sin(ang) * w[sel]
        mb = np.degrees(np.arctan2(sw.sum(), cw.sum())) % 360.0
        R = min(np.hypot(cw.sum(), sw.sum()) / w[sel].sum(), 1.0)
        out.append(dict(center=float((mb / 2.0) % wrap),
                        spread=float(np.degrees(np.sqrt(-2.0 * np.log(max(R, 1e-9)))) / 2.0),
                        frac=float(w[sel].sum() / w.sum()),
                        n=int(sel.sum())))
        used |= sel
    return out


def wrap_sym(a, period=180.0):
    """把 mod-period 的无向角折回 (-period/2, period/2]。"""
    return (np.asarray(a, float) + period / 2.0) % period - period / 2.0


def circ_std(deg, period):
    """mod-period 环形标准差（度）。"""
    ang = np.radians(np.asarray(deg, float) * (360.0 / period))
    c, s = np.cos(ang).mean(), np.sin(ang).mean()
    R = min(np.hypot(c, s), 1.0)
    return float(np.degrees(np.sqrt(-2.0 * np.log(max(R, 1e-12)))) * period / 360.0)


def circ_mean(deg, period):
    ang = np.radians(np.asarray(deg, float) * (360.0 / period))
    return float(np.degrees(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())
                            ) * period / 360.0)


# ----------------------------------------------------------------------------
# 线段提取
# ----------------------------------------------------------------------------
def extract(und, K, up_c, height, cfg):
    h, w = und.shape[:2]
    hl = horizon_line(K, up_c)
    vh = horizon_row_at(hl, w / 2.0)
    lo = int(np.clip(vh + cfg["roi_below_h"], 0, h - 1))
    hi = min(h, int(vh + cfg["roi_bottom"]))
    gray = cv2.cvtColor(und[lo:hi], cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    edges = cv2.Canny(gray, cfg["canny1"], cfg["canny2"])
    segs = cv2.HoughLinesP(edges, 1, np.pi / 720.0, cfg["hough_thr"],
                           minLineLength=cfg["min_len"], maxLineGap=cfg["max_gap"])
    basis = _horizontal_basis(K, up_c)
    res = []
    if segs is None:
        return res, lo, hi, vh
    for x1, y1, x2, y2 in np.asarray(segs).reshape(-1, 4):
        y1f, y2f = float(y1) + lo, float(y2) + lo
        a = y2f - y1f
        b = -(float(x2) - float(x1))
        c = float(x2) * y1f - float(x1) * y2f
        nrm = np.hypot(a, b)
        if nrm < 1e-9:
            continue
        al = alpha_of_line(K, up_c, np.array([a, b, c]) / nrm, basis)
        if al is None or abs(al) > cfg["max_abs_alpha"]:
            continue
        # 地距门限：两端点反投影，要求落在 [r_min, r_max]
        g1 = ground_range(K, up_c, float(x1), y1f, height)
        g2 = ground_range(K, up_c, float(x2), y2f, height)
        if g1 is None or g2 is None:
            continue
        if not (cfg["r_min"] < g1[2] < cfg["r_max"] and cfg["r_min"] < g2[2] < cfg["r_max"]):
            continue
        length = float(np.hypot(x2 - x1, y2f - y1f))
        if length < cfg["min_len"]:
            continue
        res.append((al, length, (float(x1), y1f, float(x2), y2f),
                    (g1[2], g2[2])))
    return res, lo, hi, vh


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default=os.path.join(HERE, "..", "parking01"))
    ap.add_argument("--images", default=os.path.join(HERE, "stereo_images", "left"))
    ap.add_argument("--calib", default=os.path.join(HERE, "calibration.yaml"))
    ap.add_argument("--out", default=os.path.join(HERE, "plots", "lane_probe"))
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--min-speed", type=float, default=0.30)
    ap.add_argument("--max-yawrate", type=float, default=6.0,
                    help="°/s，超过则视为转弯中，不计入一致性统计")
    ap.add_argument("--save-vis", type=int, default=6)
    ap.add_argument("--pitch-pert", type=float, default=0.0,
                    help="给 pitch 加扰动（度），做几何灵敏度分析")
    ap.add_argument("--roll-pert", type=float, default=0.0)
    ap.add_argument("--attitude", default="gt", choices=["gt", "fixed", "online", "gt_clean"],
                    help="roll/pitch 来源：gt=真值原始(含尖峰伪影)；gt_clean=真值0.5s中值滤波(评估上界)；"
                         "fixed=起始对准；online=互补滤波(可部署)")
    ap.add_argument("--att-tau", type=float, default=10.0, help="online 姿态互补滤波时间常数 s")
    ap.add_argument("--fixed-attitude", action="store_true",
                    help="（旧口径，等价 --attitude fixed）全程使用起始静止对准的 roll/pitch")
    args = ap.parse_args()

    cfg = dict(roi_below_h=8, roi_bottom=1200, canny1=60, canny2=160,
               hough_thr=50, min_len=110, max_gap=10,
               max_abs_alpha=80.0, r_min=1.0, r_max=16.0)

    cal = load_calibration(args.calib)
    K, dist = cal["K"], cal["dist"]
    # 相机离地高度：IMU 距轮速参考点 + 相机相对 IMU（z-up body）
    height = float(abs(cal["odo_lever"][2]) - cal["t_bc"][2] * 0 + abs(cal["t_bc"][2]))
    print("内参 fx=%.3f fy=%.3f cx=%.2f cy=%.2f | 相机离地高度 h=%.3f m"
          % (K[0, 0], K[1, 1], K[0, 2], K[1, 2], height))

    size = (cal["res"][0], cal["res"][1])
    map1, map2 = cv2.initUndistortRectifyMap(K, None, None, K, size, cv2.CV_32FC1)

    imu_t0 = float(open(os.path.join(args.data_dir,
                                     "parking01_ADIS16465_IMU.txt")).readline().split()[0])
    names, rel = [], []
    with open(os.path.join(args.images, "timestamps.txt")) as f:
        for ln in f:
            if ln.startswith("#") or not ln.strip():
                continue
            p = ln.split()
            names.append(p[0])
            rel.append(float(p[1]))
    t_img = imu_t0 + np.asarray(rel)

    g = np.loadtxt(os.path.join(args.data_dir, "parking01_groundtruth.nav"))
    gt_t, gt_roll, gt_pitch, gt_yaw = g[:, 0], g[:, 7], g[:, 8], g[:, 9]
    gt_v = np.hypot(g[:, 5], g[:, 6])

    def block_median(times, vals, w):
        nb = len(vals) // w
        tb = times[:nb * w].reshape(nb, w).mean(1)
        vb = np.median(vals[:nb * w].reshape(nb, w), axis=1)
        return tb, vb
    gt_tc, gt_rollc = block_median(gt_t, gt_roll, 100)
    _, gt_pitchc = block_median(gt_t, gt_pitch, 100)
    att_t = att_r = att_p = None
    if args.attitude == "online":
        import lane_heading as lh
        imud = np.loadtxt(os.path.join(args.data_dir, "parking01_ADIS16465_IMU.txt"))
        it = imud[:, 0]; gy = imud[:, 1:4] * 200.0; ac = imud[:, 4:7] * 200.0
        odod = np.loadtxt(os.path.join(args.data_dir, "parking01_RANGER_ODO.txt"))
        ot = odod[:, 0]; ov = odod[:, 1:5].mean(1)
        st = (np.abs(np.linalg.norm(ac, axis=1) - 9.80665) < 0.15) & (np.linalg.norm(gy, axis=1) < 0.02)
        st &= it < it[0] + 20.0                       # 起始静止窗口内
        d0 = -ac[st].mean(0)
        roll0 = float(np.degrees(np.arctan2(d0[1], d0[2])))
        pitch0 = float(np.degrees(np.arctan2(-d0[0], np.hypot(d0[1], d0[2]))))
        ar, ap_ = lh.run_attitude_track(it, gy, ac, roll0, pitch0,
                                        tau=args.att_tau, wheel_times=ot, wheel_speed=ov)
        att_t, att_r, att_p = it, ar, ap_
        print("online 姿态就绪: 起始 roll %.2f° pitch %.2f°, tau=%.0fs" % (roll0, pitch0, args.att_tau))

    os.makedirs(args.out, exist_ok=True)
    rows, rnames = [], []
    nvis = 0
    t0 = time.time()
    idxs = list(range(0, len(names), args.stride))
    if args.max_frames:
        idxs = idxs[:args.max_frames]

    for cnt, i in enumerate(idxs):
        t = t_img[i]
        if not (gt_t[0] <= t <= gt_t[-1]):
            continue
        sp = float(np.interp(t, gt_t, gt_v))
        if sp < args.min_speed:
            continue
        img = cv2.imread(os.path.join(args.images, names[i]))
        if img is None:
            continue
        und = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        yaw = float(np.interp(t, gt_t, gt_yaw))
        if args.attitude == "online":    # 部署可行：陀螺传播+加速度计低频修正
            roll = float(np.interp(t, att_t, att_r))
            pitch = float(np.interp(t, att_t, att_p))
        elif args.attitude == "gt_clean":  # 评估口径：剔除真值姿态尖峰
            roll = float(np.interp(t, gt_tc, gt_rollc))
            pitch = float(np.interp(t, gt_tc, gt_pitchc))
        elif args.fixed_attitude:        # 部署可行：只有起始一次对准的姿态
            roll, pitch = float(gt_roll[0]), float(gt_pitch[0])
        else:
            roll = float(np.interp(t, gt_t, gt_roll))
            pitch = float(np.interp(t, gt_t, gt_pitch))
        roll += args.roll_pert
        pitch += args.pitch_pert
        yr = (float(np.interp(t + 0.5, gt_t, gt_yaw))
              - float(np.interp(t - 0.5, gt_t, gt_yaw)))
        up_c = up_in_camera(euler_321_to_dcm(roll, pitch, yaw), cal["R_bc"])
        segs, lo, hi, vh = extract(und, K, up_c, height, cfg)
        if not segs:
            continue
        al = np.array([s[0] for s in segs])
        w = np.array([s[1] for s in segs])
        modes = _circ_modes(al, w, wrap=180.0, bin_deg=1.0, win_deg=3.0)
        if not modes:
            continue
        m1 = modes[0]
        a1 = float(wrap_sym(m1["center"]))
        a2 = float(wrap_sym(modes[1]["center"])) if len(modes) > 1 else np.nan
        orth = (abs(np.mod(modes[1]["center"] - m1["center"] + 90.0, 180.0) - 90.0)
                if len(modes) > 1 else np.nan)
        a2c, sp2, fr2, orth2 = orthogonality_check(segs, a1)
        c90 = float(wrap_sym(a1 + yaw, 90.0))
        rows.append((t, sp, yaw, yr, roll, pitch, vh, len(segs),
                     a1, m1["spread"], m1["frac"], a2, orth, c90,
                     a2c, sp2, fr2, orth2))
        rnames.append(names[i])
        if nvis < args.save_vis:
            nvis += 1
            vis = und.copy()
            for (a, L, (x1, y1, x2, y2), (r1, r2)) in segs:
                keep = abs(np.mod(a - m1["center"] + 90, 180) - 90) < 3
                cv2.line(vis, (int(x1), int(y1)), (int(x2), int(y2)),
                         (0, 255, 0) if keep else (0, 0, 255), 3)
            cv2.line(vis, (0, int(vh)), (vis.shape[1], int(vh)), (255, 0, 0), 2)
            cv2.putText(vis, "%s a1=%+.1f sp=%.2f a2=%s n=%d"
                        % (names[i], a1, m1["spread"],
                           ("%+.1f" % a2) if np.isfinite(a2) else "  --", len(segs)),
                        (12, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 3)
            cv2.imwrite(os.path.join(args.out, "seg_%02d_%s" % (nvis, names[i])), vis)
        if (cnt + 1) % 100 == 0:
            print("  %d/%d 帧 %.1f s" % (cnt + 1, len(idxs), time.time() - t0))

    arr = np.array(rows, float)
    hdr = ("t,speed,yaw_gt,yawrate,roll,pitch,v_horizon,nseg,"
           "alpha1,spread1,frac1,alpha2,orth_err,c90,"
           "alpha2_con,spread2,frac2,orth2")
    with open(os.path.join(args.out, "probe_results.csv"), "w") as f:
        f.write("frame," + hdr + "\n")
        for nm, r in zip(rnames, arr):
            f.write(nm + "," + ",".join("%.4f" % x for x in r) + "\n")
    print("\n处理 %d 帧（有效 %d 帧），%.1f s（%.0f ms/帧）"
          % (len(idxs), len(arr), time.time() - t0,
             (time.time() - t0) / max(len(arr), 1) * 1000))

    # ------------------------- 一致性统计（决定性结论） -------------------------
    lines = []
    lines.append("lane_probe 报告  (stride=%d, 有效帧=%d)" % (args.stride, len(arr)))
    lines.append("相机离地高度 h=%.3f m | pitch扰动=%+.2f° roll扰动=%+.2f° 姿态源=%s"
                 % (height, args.pitch_pert, args.roll_pert,
                    {"online": "在线互补滤波", "fixed": "固定(起始对准)"}.get(
                        "fixed" if args.fixed_attitude else args.attitude, "GT时变")))
    ok = np.abs(arr[:, 3]) < args.max_yawrate          # 排除转弯中的帧
    sel = arr[ok]
    lines.append("直行帧（|yaw rate|<%.1f°/s）: %d / %d" % (args.max_yawrate, ok.sum(), len(arr)))
    for tag, d in (("全部", arr), ("直行", sel)):
        if len(d) < 5:
            continue
        c = d[:, 13]
        lines.append("  [%s] c90=wrap90(α1+yaw) 环形均值 %+.2f°  环形标准差 %.2f°  "
                     "P68 %.2f°  最大偏 %+.2f°"
                     % (tag, circ_mean(c, 90.0), circ_std(c, 90.0),
                        np.percentile(np.abs(wrap_sym(c - circ_mean(c, 90.0), 90.0)), 68),
                        np.abs(wrap_sym(c - circ_mean(c, 90.0), 90.0)).max()))
    lines.append("  α1 段内 spread 中位数 %.2f°  P90 %.2f°"
                 % (np.median(arr[:, 9]), np.percentile(arr[:, 9], 90)))
    o2, f2 = arr[:, 17], arr[:, 16]
    lines.append("  次族正交自检（α1+90°±15° 约束搜索）: orth2 中位 %.2f°  P90 %.2f°；"
                 "次族权重占比 frac2 中位 %.2f；正交邻域无线段帧数 %.0f%%"
                 % (np.nanmedian(o2), np.nanpercentile(o2, 90), np.median(f2),
                    100.0 * np.mean(~np.isfinite(o2))))
    lines.append("  检出的 α1 分布: 中位 %+.1f°  P5 %+.1f°  P95 %+.1f°"
                 % (np.median(arr[:, 8]), np.percentile(arr[:, 8], 5),
                    np.percentile(arr[:, 8], 95)))
    lines.append("  每帧线段数 中位 %.0f  α1 众数权重占比 中位 %.2f"
                 % (np.median(arr[:, 7]), np.median(arr[:, 10])))
    rep = "\n".join(lines)
    print("\n" + rep)
    with open(os.path.join(args.out, "report.txt"), "w", encoding="utf-8") as f:
        f.write(rep + "\n")

    # ------------------------------- 出图 -------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        ax[0].plot(arr[:, 0] - arr[0, 0], wrap_sym(arr[:, 8] + arr[:, 2], 180.0),
                  ".", ms=3, label="α1 + yaw (mod180)")
        ax[0].plot(arr[ok, 0] - arr[0, 0], arr[ok, 13], "g.", ms=4, label="c90 (直行帧)")
        ax[0].axhline(circ_mean(arr[:, 13], 90.0), color="k", lw=.6)
        ax[0].legend()
        ax[0].set_ylabel("一致性量 (°)")
        ax[0].set_title("车道线方向角 α 与真值航向的一致性")
        ax[1].plot(arr[:, 0] - arr[0, 0], arr[:, 2], label="GT yaw")
        ax[1].plot(arr[:, 0] - arr[0, 0], -arr[:, 8], label="-α1")
        ax[1].legend()
        ax[1].set_xlabel("t (s)")
        ax[1].set_ylabel("航向 / 线方向 (°)")
        fig.tight_layout()
        fig.savefig(os.path.join(args.out, "alpha_vs_yaw.png"), dpi=110)
        fig2, ax2 = plt.subplots(1, 2, figsize=(11, 4))
        ax2[0].hist(wrap_sym(arr[:, 13] - circ_mean(arr[:, 13], 90.0), 90.0),
                    bins=60, range=(-45, 45))
        ax2[0].set_title("c90 残差直方图（应集中在 0）")
        ax2[0].set_xlabel("°")
        ax2[1].hist(arr[:, 9], bins=40)
        ax2[1].set_title("每帧 α1 众数内散布 (°)")
        fig2.tight_layout()
        fig2.savefig(os.path.join(args.out, "consistency.png"), dpi=110)
        print("图已输出 -> %s/{alpha_vs_yaw,consistency}.png" % args.out)
    except Exception as e:
        print("出图跳过:", e)


if __name__ == "__main__":
    main()
