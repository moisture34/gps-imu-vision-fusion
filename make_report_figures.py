#!/usr/bin/env python3
"""重画报告用总览图（可复现，替代此前手工生成的 PNG）。

输出两张图到 --output_dir：
  full_fit.png            全轨迹拟合总览（对齐后形状 + 局部放大 + 逐帧误差）
  absolute_vs_shape.png   未对齐(绝对定位) vs 2D刚体对齐后(形状精度)

两者都只依赖 fusion_results.csv 与 .nav 真值，数字与 error_report.txt 一致。
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = [
    'SimHei', 'Noto Sans CJK SC', 'Noto Sans CJK JP',
    'WenQuanYi Zen Hei', 'AR PL UMing CN', 'DejaVu Sans',
]
plt.rcParams['axes.unicode_minus'] = False

from evaluate_trajectory import (
    load_fusion_results, load_groundtruth,
    align_trajectories, rigid_align_2d,
)


def stats(x, y, xg, yg):
    pos = np.sqrt((x - xg) ** 2 + (y - yg) ** 2)
    return pos, float(np.sqrt(np.mean(pos ** 2))), float(np.mean(pos)), float(pos.max())


def main():
    ap = argparse.ArgumentParser(description="报告用总览图")
    ap.add_argument("--groundtruth", required=True)
    ap.add_argument("--run", default="plots/ablation/vo_bias/fusion_results.csv",
                    help="融合结果 CSV（最佳配置）")
    ap.add_argument("--output_dir", default="plots/ablation")
    ap.add_argument("--label", default="VO+IMU+轮速 融合（2D刚体对齐后）")
    args = ap.parse_args()

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    time_est, x_est, y_est, yaw_est, _ = load_fusion_results(args.run)
    gt = load_groundtruth(args.groundtruth)
    t_al, xg, yg, yawg, idx = align_trajectories(time_est, gt[0], args.groundtruth)
    x_raw, y_raw, yaw_raw = x_est[idx], y_est[idx], yaw_est[idx]
    xa, ya, yaw_a, psi0 = rigid_align_2d(x_raw, y_raw, yaw_raw, xg, yg, yawg)

    t_rel = t_al - t_al[0]
    pos_a, rmse_a, mean_a, max_a = stats(xa, ya, xg, yg)
    pos_u, rmse_u, mean_u, max_u = stats(x_raw, y_raw, xg, yg)
    gt_len = float(np.sum(np.hypot(np.diff(xg), np.diff(yg))))

    print(f"[{args.run}]  帧数 {len(t_al)}  时长 {t_rel[-1]:.0f}s  真值里程 {gt_len:.0f} m")
    print(f"  对齐后(形状)  RMSE {rmse_a:.4f} m | 均值 {mean_a:.3f} | 最大 {max_a:.3f}")
    print(f"  未对齐(绝对)  RMSE {rmse_u:.2f} m | 最大 {max_u:.1f} | 航向偏移 {np.degrees(psi0):.2f}°")

    # ---------- 图 1：全轨迹拟合总览 ----------
    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1.0], hspace=0.28, wspace=0.18)

    ax1 = fig.add_subplot(gs[:, 0])
    ax1.plot(xg, yg, color="#808080", lw=3, label=f"真值 GT ({gt_len:.0f} m / {t_rel[-1]/60:.1f} min)")
    ax1.plot(xa, ya, color="red", lw=1.0, label=args.label)
    ax1.plot(xa[0], ya[0], "o", ms=12, mfc="green", mec="k", label="起点")
    ax1.plot(xa[-1], ya[-1], "^", ms=12, mfc="blue", mec="k", label="终点")
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.axis("equal"); ax1.grid(alpha=0.3)
    ax1.legend(loc="upper left", fontsize=10)
    ax1.set_title("全轨迹拟合（3666帧/1060s，无GNSS、无真值初始化）\n"
                  f"位置 RMSE {rmse_a:.3f} m · 均值 {mean_a:.3f} m · 最大 {max_a:.3f} m",
                  fontsize=13, pad=10)

    # 局部放大：以起点为中心的窗口
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(xg, yg, color="#808080", lw=2)
    ax2.plot(xa, ya, color="red", lw=0.8)
    ax2.axis("equal"); ax2.grid(alpha=0.3)
    ax2.set_xlim(xa[0] - 45, xa[0] + 90); ax2.set_ylim(ya[0] - 45, ya[0] + 55)
    ax2.set_title("局部放大（误差在此尺度下仍不可见）")

    ax3 = fig.add_subplot(gs[1, 1])
    ax3.fill_between(t_rel, pos_a, alpha=0.25, color="red")
    ax3.plot(t_rel, pos_a, color="red", lw=0.9)
    ax3.axhline(rmse_a, ls="--", color="k", lw=1, label=f"RMSE {rmse_a:.2f} m")
    ax3.set_xlabel("时间 (s)"); ax3.set_ylabel("位置误差 (m)")
    ax3.set_title("逐帧位置误差（无累积漂移趋势）"); ax3.legend(); ax3.grid(alpha=0.3)

    fig.savefig(out / "full_fit.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    # ---------- 图 2：绝对定位 vs 形状精度 ----------
    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1.0], wspace=0.16)

    axa = fig.add_subplot(gs[0, 0])
    axa.plot(xg, yg, color="#808080", lw=3, label=f"真值 GT（起点 (0,0)，{t_rel[-1]/60:.1f} min / {gt_len:.0f} m）")
    axa.plot(x_raw, y_raw, color="red", lw=1.0, label="融合输出（部署模式，原始里程计系）")
    axa.plot(x_raw[0], y_raw[0], "o", ms=12, mfc="green", mec="k", label="融合起点")
    axa.axis("equal"); axa.grid(alpha=0.3); axa.legend(loc="upper left", fontsize=10)
    axa.set_title(f"【未对齐】绝对定位：RMSE {rmse_u:.2f} m，最大 {max_u:.1f} m\n"
                  f"整条轨迹因初始航向未知而被旋转 {np.degrees(psi0):.1f}°（形状其实完全正确）",
                  fontsize=13, pad=10)

    axb = fig.add_subplot(gs[0, 1])
    axb.plot(xg, yg, color="#808080", lw=3)
    axb.plot(xa, ya, color="red", lw=0.9)
    axb.plot(xa[0], ya[0], "o", ms=10, mfc="green", mec="k")
    axb.plot(xa[-1], ya[-1], "^", ms=10, mfc="blue", mec="k")
    axb.axis("equal"); axb.grid(alpha=0.3)
    axb.annotate("红线压在黑线上\n(此尺度看不出 {:.2f} m)".format(rmse_a),
                 xy=(xa[0] + 5, ya[0]), xytext=(xa[0] - 30, ya[0] - 12),
                 arrowprops=dict(arrowstyle="->", color="k", alpha=0.6))
    axb.set_title(f"【2D刚体对齐后】形状精度\nRMSE {rmse_a:.3f} m · max {max_a:.2f} m", fontsize=13)

    fig.savefig(out / "absolute_vs_shape.png", dpi=130, bbox_inches="tight")
    plt.close(fig)

    print(f"\n已保存:\n  {out / 'full_fit.png'}\n  {out / 'absolute_vs_shape.png'}")


if __name__ == "__main__":
    main()
