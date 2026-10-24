#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
heading_lever.py —— 量化"绝对航向误差 → 绝对定位误差"的杠杆（README 引用来源）

把已归档的融合轨迹绕其起点旋转不同角度 δ，与真值直接比较（**不做任何对齐**），
得到"若绝对航向误差为 δ，未对齐绝对 RMSE 是多少"。用于论证 P0-2 的价值：
车道线航向观测把绝对航向误差从 87.42° 压到亚度级时，绝对定位误差的落点。

用法： .venv/bin/python3 heading_lever.py
输出： plots/lane_probe/heading_lever.txt
"""
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "plots", "ablation", "vo_bias", "fusion_results.csv")
NAV = os.path.join(HERE, "..", "parking01", "parking01_groundtruth.nav")


def main():
    d = np.loadtxt(CSV, delimiter=",", skiprows=1)
    t, est = d[:, 0], d[:, 1:3]
    g = np.loadtxt(NAV, usecols=(0, 1, 2))
    gt = np.stack([np.interp(t, g[:, 0], g[:, 1]), np.interp(t, g[:, 0], g[:, 2])], 1)
    ok = (g[0, 0] <= t) & (t <= g[-1, 0])
    est, gt = est[ok], gt[ok]
    sh = est - est[0]

    def rmse(delta_deg):
        a = np.deg2rad(delta_deg)
        R = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        return float(np.sqrt(((sh @ R.T + gt[0] - gt) ** 2).sum(1).mean()))

    grid = np.linspace(-180, 180, 14401)
    vals = np.array([rmse(x) for x in grid])
    best = float(grid[vals.argmin()])
    r = np.linalg.norm(gt - gt[0], axis=1)
    lever = float(np.sqrt((r ** 2).mean()) * np.pi / 180.0)

    lines = ["绝对航向误差 -> 未对齐绝对定位误差（绕起点旋转归档融合轨迹，不做对齐）", "=" * 74]
    lines.append("输入: %s" % os.path.relpath(CSV, HERE))
    lines.append("评估帧数 %d | 原始（完全不对齐）RMSE = %.2f m" %
                 (len(est), float(np.sqrt(((est - gt) ** 2).sum(1).mean()))))
    lines.append("最优旋转角 = %.2f°（实测航向偏置）, 此时 RMSE = %.3f m（纯形状误差下限）"
                 % (best, rmse(best)))
    lines.append("轨迹到起点距离: 均值 %.1f m 最大 %.1f m -> 杠杆 %.2f m/°"
                 % (r.mean(), r.max(), lever))
    lines.append("-" * 74)
    lines.append("%22s %18s" % ("残余绝对航向误差", "未对齐绝对 RMSE (m)"))
    for x in [0.0, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0]:
        lines.append("%18.2f° %16.2f" % (x, rmse(best + x)))
    lines.append("%18.2f° %16.2f  <- 现状：完全不修正航向" % (abs(best), rmse(0.0)))
    lines.append("注：本表把估计轨迹绕其起点旋转，故 0 度行的 1.21 m 是『旋转+原点共用』的下限；")
    lines.append("    2D 刚体对齐（可同时优化平移）给出的形状误差 0.888 m 更低，两者口径不同。")
    txt = "\n".join(lines)
    print(txt)
    out = os.path.join(HERE, "plots", "lane_probe", "heading_lever.txt")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print("->", out)


if __name__ == "__main__":
    main()
