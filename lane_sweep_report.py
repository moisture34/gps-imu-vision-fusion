#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""lane_sweep_report.py —— P0-2 落地全配置对比表（R 扫描 + 象限错配对照）。

统一自算口径（所有行可比、可复现）：
  shapePos : 2D 刚体对齐（旋转+平移，无尺度）后的位置 RMSE
  shYaw    : 解卷绕偏航差去除中位常数后的 RMSE（形状偏航）
  absPos   : 不对齐、直接在 GT 帧求差的位置 RMSE（绝对定位主指标）
  absYaw   : 不对齐偏航 RMSE（绝对模式的核心卖点）
并附车道通道统计（尝试/门内/采纳）与日志里的初始化来源、C_lane、NIS 摘要。
"""
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
GT = os.path.join(HERE, "..", "parking01", "parking01_groundtruth.nav")

RUNS = [
    ("基线", "plots/ablation/vo_bias"),
    ("grid", "plots/lane_full/grid"),
    ("abs_k1", "plots/lane_full/abs_k1"),
    ("abs_R07", "plots/lane_full/abs_R07"),
    ("abs_R12", "plots/lane_full/abs_R12"),
    ("abs_adaptR", "plots/lane_full/abs_adaptR"),
    ("grid_R07", "plots/lane_full/grid_R07"),
    ("abs_k0", "plots/lane_full/abs_k0"),
    ("abs_initonly", "plots/lane_full/abs_initonly"),
    ("abs_gyr25", "plots/lane_full/abs_gyr25"),
    ("abs_qfix", "plots/lane_full/abs_qfix"),
    ("base_qfix", "plots/lane_full/base_qfix"),
    ("abs_g25_qfix", "plots/lane_full/abs_g25_qfix"),
    ("grid_g25_qfix", "plots/lane_full/grid_g25_qfix"),
]


def rigid_align2d(pe, pg):
    ce, cg = pe.mean(0), pg.mean(0)
    A, B = pe - ce, pg - cg
    th = np.arctan2((A[:, 0] * B[:, 1] - A[:, 1] * B[:, 0]).sum(), (A * B).sum())
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return (R @ pe.T).T + cg - R @ ce


def main():
    g = np.loadtxt(GT)
    gt = g[:, 0]
    gx = np.interp(gt, gt, g[:, 1])
    gy = np.interp(gt, gt, g[:, 2])
    gyaw = np.degrees(np.unwrap(np.radians(g[:, 9])))
    rows = []
    for tag, rel in RUNS:
        p = os.path.join(HERE, rel)
        f = np.loadtxt(os.path.join(p, "fusion_results.csv"),
                       delimiter=",", skiprows=1)
        t = f[:, 0]
        m = (t >= gt[0]) & (t <= gt[-1])
        t, x, y, yaw = t[m], f[m, 1], f[m, 2], f[m, 3]
        XG = np.interp(t, gt, gx)
        YG = np.interp(t, gt, gy)
        pa = rigid_align2d(np.c_[x, y], np.c_[XG, YG])
        shape_pos = float(np.sqrt(((pa[:, 0] - XG) ** 2 + (pa[:, 1] - YG) ** 2).mean()))
        abs_pos = float(np.sqrt(((x - XG) ** 2 + (y - YG) ** 2).mean()))
        e = np.mod(np.degrees(np.unwrap(yaw)) - np.interp(t, gt, gyaw)
                   + 180.0, 360.0) - 180.0
        sh_yaw = float(np.sqrt(((e - np.median(e)) ** 2).mean()))
        ab_yaw = float(np.sqrt((e ** 2).mean()))
        at = ok = ac = 0
        lc = os.path.join(p, "lane_obs_results.csv")
        if os.path.exists(lc):
            r = np.loadtxt(lc, delimiter=",", skiprows=1)
            at, ok, ac = len(r), int(r[:, 7].sum()), int(r[:, 8].sum())
        note = ""
        log = os.path.join(HERE, "plots", "lane_full", tag + ".log")
        if os.path.exists(log):
            tx = open(log, encoding="utf-8", errors="ignore").read()
            mm = re.findall(r"绝对偏航初始化\((\S+?),", tx)
            if mm:
                note += "init:" + mm[0]
            c = re.findall(r"C_lane: ([+\-\d.]+)°", tx)
            if c:
                note += " C=%.2f°" % float(c[-1])
            ni = re.findall(r"车道航向 NIS: 均值 ([\d.]+).*?拒绝 (\d+) 次", tx, re.S)
            if ni:
                note += " NIS均%s 拒%s" % ni[-1]
            u = re.findall(r"更新: (\d+) 次通过", tx)
            if u:
                note += " 通过%s" % u[-1]
        rows.append((tag, shape_pos, sh_yaw, abs_pos, ab_yaw, at, ok, ac, note))

    hdr = ("%-11s %8s %6s %8s %7s  %5s %5s %5s  %s"
           % ("run", "shapePos", "shYaw", "absPos", "absYaw",
              "尝试", "门内", "采纳", "备注"))
    lines = [hdr, "-" * len(hdr)]
    for r in rows:
        lines.append("%-11s %8.3f %6.2f %8.2f %7.2f  %5d %5d %5d  %s" % r)
    txt = "\n".join(lines)
    print(txt)
    out = os.path.join(HERE, "plots", "lane_full", "lane_sweep_report.txt")
    with open(out, "w", encoding="utf-8") as fp:
        fp.write(txt + "\n")


if __name__ == "__main__":
    main()
