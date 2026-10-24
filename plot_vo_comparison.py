#!/mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main/.venv/bin/python3
"""VO 融合 vs 无VO 基线 对比图。

读取多套融合结果 CSV（例如"含双目/单目VO"与"纯IMU+轮速无VO"），
各自做时间对齐 + 2D 刚体对齐后，与真值一起画出：
  - 轨迹对比（对齐后形状）
  - 逐帧位置误差
并打印每套的位置/偏航 RMSE。
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


def summarize(x_est, y_est, yaw_est, x_gt, y_gt, yaw_gt):
    pos = np.sqrt((x_est - x_gt) ** 2 + (y_est - y_gt) ** 2)
    dyaw = np.arctan2(np.sin(yaw_est - yaw_gt), np.cos(yaw_est - yaw_gt))
    return {
        "rmse_pos": float(np.sqrt(np.mean(pos ** 2))),
        "mean_pos": float(np.mean(pos)),
        "max_pos": float(np.max(pos)),
        "rmse_yaw": float(np.degrees(np.sqrt(np.mean(dyaw ** 2)))),
        "pos_curve": pos,
    }


def main():
    ap = argparse.ArgumentParser(description="VO vs 无VO 对比图")
    ap.add_argument("--groundtruth", required=True)
    ap.add_argument("--results", nargs="+", required=True,
                    help="结果CSV，格式 label=path")
    ap.add_argument("--labels", nargs="+", default=None,
                    help="与 --results 对应的显示名（可省略则用 label）")
    ap.add_argument("--output_dir", default="plots/comparison")
    ap.add_argument("--prefix", default="vo_vs_baseline",
                    help="输出图文件名前缀，便于多组对比图并存")
    args = ap.parse_args()

    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    t_gt, x_gt, y_gt, *_ , yaw_gt = load_groundtruth(args.groundtruth)

    colors = ["tab:red", "tab:blue", "tab:green", "tab:orange"]
    fig_traj, ax1 = plt.subplots(figsize=(9, 8))
    fig_err, ax2 = plt.subplots(figsize=(11, 5))

    print("=" * 60)
    print("VO 融合 vs 无VO 基线  (各自 2D 刚体对齐后度量)")
    print("=" * 60)

    for i, spec in enumerate(args.results):
        if "=" in spec:
            label, csv_path = spec.split("=", 1)
        else:
            label, csv_path = f"run{i}", spec
        disp = args.labels[i] if args.labels and i < len(args.labels) else label
        t, x, y, yaw, _ = load_fusion_results(csv_path)
        ta, xg, yg, yawg, idx = align_trajectories(t, t_gt, args.groundtruth)
        xa, ya, yawa, theta = rigid_align_2d(x[idx], y[idx], yaw[idx], xg, yg, yawg)
        s = summarize(xa, ya, yawa, xg, yg, yawg)
        c = colors[i % len(colors)]

        ax1.plot(xg, yg, "k-", lw=2.5, label="Ground Truth" if i == 0 else None)
        ax1.plot(xa, ya, "-", color=c, lw=1.6,
                 label=f"{disp} (RMSE {s['rmse_pos']:.2f} m, {s['rmse_yaw']:.1f}°)")
        ax2.plot(ta - ta[0], s["pos_curve"], "-", color=c, lw=1.3, label=disp)

        print(f"\n[{disp}]  {csv_path}")
        print(f"  位置 RMSE {s['rmse_pos']:.3f} m | 均值 {s['mean_pos']:.3f} m | 最大 {s['max_pos']:.3f} m")
        print(f"  偏航 RMSE {s['rmse_yaw']:.3f}°  | 对齐航向偏移 {np.degrees(theta):.2f}°")

    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_title("轨迹对比（2D刚体对齐后）")
    ax1.legend(); ax1.grid(True, alpha=0.3); ax1.axis("equal")
    fig_traj.tight_layout(); fig_traj.savefig(out / (args.prefix + "_trajectory.png"), dpi=130)

    ax2.set_xlabel("时间 (s)"); ax2.set_ylabel("位置误差 (m)")
    ax2.set_title("逐帧位置误差（对齐后）"); ax2.legend(); ax2.grid(True, alpha=0.3)
    fig_err.tight_layout(); fig_err.savefig(out / (args.prefix + "_error.png"), dpi=130)

    print(f"\n对比图已保存:\n  {out / (args.prefix + '_trajectory.png')}\n  {out / (args.prefix + '_error.png')}")


if __name__ == "__main__":
    main()
