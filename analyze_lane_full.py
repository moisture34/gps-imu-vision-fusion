#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_lane_full.py —— P0-2 全量落地结果汇总（基线 / 自洽模式 / 绝对模式）

统一自算口径（不依赖 evaluate_trajectory 的 --align 标志，全部在同一脚本内计算，
保证三行数字可比）：
  * 不对齐绝对 RMSE：融合 (x,y) 与 GT 同时刻直接求差（GT 帧；起点按构造重合）。
  * 对齐形状 RMSE / 偏航形状 RMSE：2D 刚体对齐（旋转+平移，无尺度）后求差。
  * 绝对偏航误差（仅绝对模式有意义）：wrap(ψ_est − ψ_GT)，不做任何对齐。
产物：plots/lane_full/lane_full_report.txt + lane_full_summary.png
"""
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GT_FILE = os.path.join(HERE, "..", "parking01", "parking01_groundtruth.nav")

RUNS = [
    ("基线(VO+零偏)", os.path.join(HERE, "plots", "ablation", "vo_bias"), None),
    ("车道·自洽", os.path.join(HERE, "plots", "lane_full", "grid_g25_qfix"),
     os.path.join(HERE, "plots", "lane_full", "grid_g25_qfix", "lane_obs_results.csv")),
    ("车道·绝对k=1", os.path.join(HERE, "plots", "lane_full", "abs_g25_qfix"),
     os.path.join(HERE, "plots", "lane_full", "abs_g25_qfix", "lane_obs_results.csv")),
]


def load_gt():
    g = np.loadtxt(GT_FILE)
    return g[:, 0], g[:, 1], g[:, 2], np.unwrap(np.radians(g[:, 9]))


def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0


def rigid_align2d(pe, pg):
    """无尺度刚体对齐：返回 (theta, tx, ty)。pe/pg: (N,2) 同索引对应点。"""
    ce, cg = pe.mean(0), pg.mean(0)
    A = pe - ce
    B = pg - cg
    num = (A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum()
    den = (A * B).sum()
    th = np.arctan2(num, den)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    t = cg - R @ ce
    return th, t


def evaluate(tag, run_dir, lane_csv):
    p = os.path.join(run_dir, "fusion_results.csv")
    if not os.path.exists(p):
        return None
    d = np.loadtxt(p, delimiter=",", skiprows=1)
    t, x, y, yaw = d[:, 0], d[:, 1], d[:, 2], d[:, 3]
    gt_t, gx, gy, gyaw = load_gt()
    m = (t >= gt_t[0]) & (t <= gt_t[-1])
    t, x, y, yaw = t[m], x[m], y[m], yaw[m]
    Xg = np.interp(t, gt_t, gx)
    Yg = np.interp(t, gt_t, gy)
    YWg = np.degrees(np.interp(t, gt_t, gyaw))
    out = {"tag": tag, "n": len(t), "dur": t[-1] - t[0]}
    # —— 不对齐（绝对帧）——
    e_abs = np.hypot(x - Xg, y - Yg)
    out["abs_pos_rmse"] = float(np.sqrt((e_abs ** 2).mean()))
    out["abs_yaw_rmse"] = float(np.sqrt((wrap180(np.degrees(yaw) - YWg) ** 2).mean()))
    # —— 2D 刚体对齐（形状）——
    th, tt = rigid_align2d(np.c_[x, y], np.c_[Xg, Yg])
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    pa = (R @ np.c_[x, y].T).T + tt
    e_sh = np.hypot(pa[:, 0] - Xg, pa[:, 1] - Yg)
    out["shape_pos_rmse"] = float(np.sqrt((e_sh ** 2).mean()))
    yaw_off = np.median(wrap180(np.degrees(yaw) - YWg))
    out["align_theta_deg"] = float(np.degrees(th))
    out["shape_yaw_rmse"] = float(np.sqrt((wrap180(np.degrees(yaw) - yaw_off - YWg) ** 2).mean()))
    out["err_series"] = (t, e_abs, e_sh, wrap180(np.degrees(yaw) - YWg))
    # —— 车道通道统计 ——
    if lane_csv and os.path.exists(lane_csv):
        rec = np.loadtxt(lane_csv, delimiter=",", skiprows=1)
        out["lane"] = dict(attempt=len(rec), ok=int(rec[:, 7].sum()),
                           acc=int(rec[:, 8].sum()),
                           acc_ratio=float(rec[:, 8].mean()),
                           alpha_med=float(np.median(rec[rec[:, 7] > 0, 3])),
                           spread_med=float(np.median(rec[rec[:, 7] > 0, 4])))
    else:
        out["lane"] = None
    # log 里的 C_lane / 末端协方差行
    log = os.path.join(run_dir, "..", os.path.basename(run_dir) + ".log")
    if os.path.exists(log):
        txt = open(log, encoding="utf-8", errors="ignore").read()
        for key, pat in (("C_lane", r"C_lane: ([+\-\d.]+)°"),
                         ("yaw_cov_end", r"末端协方差 1σ: pos_x [\d.]+ m, pos_y [\d.]+ m, yaw ([\d.]+)°"),
                         ("yaw_cov_peak", r"全程最大不确定度 1σ: pos_x [\d.]+ m, pos_y [\d.]+ m, yaw ([\d.]+)°"),
                         ("lane_updates_log", r"更新: (\d+) 次通过")):
            import re
            mm = re.findall(pat, txt)
            if mm:
                out.setdefault("log", {})[key] = mm[-1]
    return out


def main():
    res = [r for r in (evaluate(*run) for run in RUNS) if r]
    lines = []
    lines.append("P0-2 全量落地结果（自算统一口径，全部可复现）")
    lines.append("")
    hdr = ("%-12s %5s %6s %8s %8s %9s %9s  %s" %
           ("运行", "帧数", "时长s", "绝对位置", "绝对偏航", "形状位置", "形状偏航", "车道通道 尝试/门内/采纳"))
    lines.append(hdr)
    lines.append("-" * len(hdr))
    for r in res:
        lane = ("  %d/%d/%d" % (r["lane"]["attempt"], r["lane"]["ok"], r["lane"]["acc"])
                if r["lane"] else "        ——")
        lines.append("%-12s %5d %6.0f %8.2f %8.2f %9.3f %9.3f %s" %
                     (r["tag"], r["n"], r["dur"], r["abs_pos_rmse"], r["abs_yaw_rmse"],
                      r["shape_pos_rmse"], r["shape_yaw_rmse"], lane))
    for r in res:
        extra = []
        if r["lane"]:
            extra.append("α中位 %+.2f° 散布中位 %.2f°" %
                         (r["lane"]["alpha_med"], r["lane"]["spread_med"]))
        if "log" in r:
            extra += ["%s=%s°" % (k, v) for k, v in r["log"].items()]
        if extra:
            lines.append("  · %s: %s" % (r["tag"], "；".join(extra)))
    txt = "\n".join(lines)
    print(txt)
    os.makedirs(os.path.join(HERE, "plots", "lane_full"), exist_ok=True)
    with open(os.path.join(HERE, "plots", "lane_full", "lane_full_report.txt"),
              "w", encoding="utf-8") as f:
        f.write(txt + "\n")

    # ------------------------------- 出图 -------------------------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = [
            "SimHei", "Noto Sans CJK SC", "Noto Sans CJK JP",
            "WenQuanYi Zen Hei", "AR PL UMing CN", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        gt_t, gx, gy, _ = load_gt()
        fig, ax = plt.subplots(1, 2, figsize=(15, 6.2))
        ax[0].plot(gx, gy, "k-", lw=1.6, label="真值 GT")
        styles = {"基线(VO+零偏)": ("tab:red", "对齐后"),
                  "车道·自洽": ("tab:orange", "对齐后"),
                  "车道·绝对k=1": ("tab:green", "绝对系·未对齐")}
        for tag, run_dir, _csv in RUNS:
            p = os.path.join(run_dir, "fusion_results.csv")
            if not os.path.exists(p):
                continue
            t, x, y = np.loadtxt(p, delimiter=",", skiprows=1,
                                 usecols=(0, 1, 2), unpack=True)
            m = (t >= gt_t[0]) & (t <= gt_t[-1])
            t, x, y = t[m], x[m], y[m]
            Xg = np.interp(t, gt_t, gx)
            Yg = np.interp(t, gt_t, gy)
            if tag == "车道·绝对k=1":
                px, py = x, y
            else:
                th, tt = rigid_align2d(np.c_[x, y], np.c_[Xg, Yg])
                R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
                pa = (R @ np.c_[x, y].T).T + tt
                px, py = pa[:, 0], pa[:, 1]
            c, mode = styles.get(tag, ("gray", ""))
            ax[0].plot(px, py, "-", color=c, lw=1.2, label="%s (%s)" % (tag, mode))
        ax[0].legend(fontsize=8)
        ax[0].set_aspect("equal")
        ax[0].set_title("轨迹：绝对模式下车道航向把整条轨迹锁进 GT 帧")
        for r in res:
            t, e_abs, e_sh, eyaw = r["err_series"]
            ax[1].plot(t - t[0], e_abs, lw=1.0,
                       label="%s 不对齐 (%.1f m)" % (r["tag"], r["abs_pos_rmse"]))
        ax[1].set_yscale("log")
        ax[1].set_xlabel("t (s)")
        ax[1].set_ylabel("位置误差 (m, log)")
        ax[1].set_title("未对齐绝对位置误差时间历程")
        ax[1].legend(fontsize=8)
        ax[1].grid(alpha=.3)
        fig.tight_layout()
        out = os.path.join(HERE, "plots", "lane_full", "lane_full_summary.png")
        fig.savefig(out, dpi=110)
        print("->", out)
    except Exception as e:
        print("出图跳过:", e)


if __name__ == "__main__":
    main()
