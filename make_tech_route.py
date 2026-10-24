#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_tech_route.py —— 论文风格总体技术路线图（申报书 图 1）。

版式仿学术论文技术路线：左侧分阶段虚线框（a~e 五个层次），自上而下数据流，
三条观测支路汇入平面 EKF，对照/负结果支路用虚线，创新通道（P0-2 车道航向）加粗描边。
输出 300 dpi PNG（Word/打印用）与矢量 PDF（投稿用）。
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle

plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK JP", "Noto Sans CJK SC", "SimHei",
    "AR PL UMing CN", "DejaVu Sans",
]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))

# ---- 学术低饱和配色 --------------------------------------------------------
C_DATA = ("#EAF1F8", "#4A6F94")      # 数据层
C_INIT = ("#FDF3E3", "#B07E33")      # 初始化
C_VO   = ("#EAF6EC", "#43794F")      # 视觉支路
C_IMU  = ("#F2F4F4", "#66727A")      # 惯性/轮速
C_LANE = ("#EFEAF7", "#5B479E")      # 车道通道（创新）
C_EKF  = ("#FBEAE6", "#B0523F")      # 融合核心
C_EVAL = ("#EDF2F4", "#3F6E80")      # 评估输出
C_NEG  = ("#FFFFFF", "#B0523F")      # 负结果对照（虚线）

fig, ax = plt.subplots(figsize=(12.6, 16.8))
ax.set_xlim(0, 100)
ax.set_ylim(0, 156.5)
ax.axis("off")
# 标题放在坐标区内、紧贴 a 层虚框上方（避免 figure 坐标造成的空档）
ax.text(50, 155.6, "总体技术路线", ha="center", va="top",
        fontsize=15, fontweight="bold")


def box(x, y, w, h, title, lines, fc, ec, lw=1.4, fs_t=11.5, fs_b=8.6,
        dashed=False, badge=None):
    """矩形流程块（y 为底边）：标题加粗，其下小字说明；badge 为右上角小标签。"""
    st = "round,pad=0.6,rounding_size=1.2"
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle=st, fc=fc, ec=ec,
                                lw=lw, linestyle="--" if dashed else "-"))
    ax.text(x + w / 2, y + h - 2.6, title, ha="center", va="top",
            fontsize=fs_t, fontweight="bold", color="#20262B")
    if lines:
        ax.text(x + w / 2, y + h - 6.6, "\n".join(lines), ha="center", va="top",
                fontsize=fs_b, color="#333B42", linespacing=1.5)
    if badge:
        ax.text(x + w - 0.6, y + h + 1.4, badge, ha="right", va="bottom",
                fontsize=9, fontweight="bold", color=ec)


def stage(y0, y1, tag, name):
    """左侧阶段虚线框 + 旋转标签（论文技术路线惯例）。"""
    ax.add_patch(Rectangle((1.5, y0), 97, y1 - y0, fill=False, ec="#9AA6AE",
                           lw=1.0, ls=(0, (6, 4))))
    ax.text(3.1, (y0 + y1) / 2, f"{tag}  {name}", rotation=90, ha="center",
            va="center", fontsize=10.5, fontweight="bold", color="#5A6670")


def arrow(p0, p1, rad=0.0, dashed=False, color="#39424A", lw=1.6):
    ax.add_patch(FancyArrowPatch(p0, p1, connectionstyle=f"arc3,rad={rad}",
                                 arrowstyle="-|>,head_length=7,head_width=4",
                                 mutation_scale=1, lw=lw, color=color,
                                 linestyle="--" if dashed else "-"))

# ================= a. 数据层 =================
stage(123.5, 151, "a", "数据层")
box(30, 138.5, 40, 11, "ROS bag：parking01（真实停车场 18.3 min）",
    ["每 3 帧取 1 帧采样；内/外参取 calibration.yaml 实测标定"], C_DATA[0], C_DATA[1])
box(7.5, 125.5, 26.5, 11.5, "左目图像 10 Hz",
    ["逐帧 Header.stamp（与 IMU 同源时钟）", "→ 真实时间戳表"], C_DATA[0], C_DATA[1])
box(37, 125.5, 26, 11.5, "ADIS16465 IMU 200 Hz",
    ["gz（含零偏）/ax/ay", "×200 换算 SI"], C_DATA[0], C_DATA[1])
box(64, 125.5, 28.5, 11.5, "RANGER 轮速计 50 Hz",
    ["四轮通道前向速度", "轮距 0.364 m"], C_DATA[0], C_DATA[1])
arrow((42, 138.5), (20.8, 137.4), rad=-0.15)
arrow((50, 138.5), (50, 137.4))
arrow((58, 138.5), (78.2, 137.4), rad=0.15)

# ================= b. 初始化 =================
stage(109, 123.2, "b", "初始化（不读真值）")
box(18, 111, 64, 11.5, "静止段（31.79 s）惯性对准",
    ["重力方向 → roll/pitch（0.16°/−1.49°）；陀螺均值 → gz 零偏 +0.0403°/s",
     "与真值反解最优零偏仅差 5%　——　保证可部署性"], C_INIT[0], C_INIT[1])
arrow((20.8, 125.5), (40, 122.7), rad=-0.1)
arrow((50, 125.5), (50, 122.7))
arrow((78.2, 125.5), (60, 122.7), rad=0.1)

# ================= c. 观测支路 =================
stage(57, 107.5, "c", "三条独立观测支路")
# c1 视觉里程计
box(7.5, 80, 26.5, 25.5, "单目视觉里程计（VO）",
    ["SuperPoint 特征 + LightGlue 匹配",
     "findEssentialMat → recoverPose",
     "→ 相对旋转 R + 平移方向 t",
     "米制尺度：轮速位移定标",
     "成功率 97.76%（3524 帧）"], C_VO[0], C_VO[1])
box(7.5, 69.8, 26.5, 8.0, "备选：DA 米制深度 PnP",
    ["实测劣化 4×，保留为负结果对照（7.3.4）"],
    C_NEG[0], C_NEG[1], dashed=True, lw=1.1, fs_t=9.4, fs_b=8.0)
arrow((20.8, 77.8), (20.8, 80), dashed=True, color="#B0523F", lw=1.1)
# c2 轮速
box(38, 82, 24, 23.5, "轮速前向速度",
    ["直接观测量 v（米制）", "同时为 VO 平移定标",
     "更新 EKF 速度状态", "结构性近零新息 →", "分道 NIS 统计修正"],
    C_IMU[0], C_IMU[1])
# c3 车道航向（创新）
box(65.5, 78.5, 28.5, 27, "车道网格航向通道",
    ["直行段车道线 Hough 提取", "在线 roll/pitch 门限（P68 0.38°）",
     "α：车道-车轴夹角（mod 90°）", "绝对模式：ψ = C0 − α",
     "自洽模式：mod-90 环绕（免绝对信息）"],
    C_LANE[0], C_LANE[1], lw=2.2, badge="创新 P0-2")
box(65.5, 66.6, 28.5, 9.8, "开机一次性输入",
    ["静止段 α 扫描零位移换系", "场地常数 C0 + 象限 k"], C_LANE[0], C_LANE[1],
    dashed=True, lw=1.1, fs_t=9.4, fs_b=8.0)
arrow((79.8, 76.4), (79.8, 78.5), color="#5B479E", lw=1.1)
arrow((20.8, 111), (20.8, 105.7), rad=0.1)
arrow((50, 111), (50, 105.7))
arrow((79.8, 111), (79.8, 105.7), rad=-0.1)

# ================= d. EKF =================
stage(28.5, 56, "d", "融合核心：平面 EKF")
box(13, 31.5, 74, 22.5, "平面约束五状态 EKF　x = [x, y, θ, v, b_g]$^\\mathrm{T}$",
    ["预测：IMU 率子步（50 Hz）去零偏积分，梯形中值抗混叠",
     "① 轮速更新 v；② 视觉位置更新——偏航保护：K 的 θ 行置零 + Joseph 协方差",
     "③ 车道航向更新：mod-90 创新 + NIS 门控（χ²），经 P 耦合修正 b_g",
     "状态增广第 6 维 C_lane（自洽模式网格相位）"], C_EKF[0], C_EKF[1], lw=1.8)
arrow((20.8, 80), (27, 54), rad=0.05)
arrow((50, 82), (50, 54))
arrow((79.8, 78.5), (73, 54), rad=-0.05)

# ================= e. 评估输出 =================
stage(2, 27.5, "e", "输出与双口径评估")
box(6, 4.5, 40, 20.5, "评测协议（双口径并报）",
    ["形状口径：2D 刚体对齐 → 0.888 m",
     "绝对口径：未对齐直接对 GT",
     "　　└ 无车道 64.35 m → 车道闭环 1.34 m",
     "2×2 主消融 + 零图像信息增益归因探针",
     "协方差一致性（偏航 1σ 峰 1.10°）"],
    C_EVAL[0], C_EVAL[1])
box(52, 4.5, 42, 20.5, "头条结果",
    ["单目+IMU+轮速最低配置：形状 0.888 m（优于官方单目 VIO）",
     "+车道通道：绝对 1.34 m / 偏航 0.39°，零 GNSS 零地图",
     "偏航 RMSE 1.87°；漂移斜率 +0.41 mm/s（无累积）",
     "位级可复现；全部结论落盘可核"], C_EVAL[0], C_EVAL[1], lw=1.8)
arrow((40, 31.5), (26, 25), rad=0.1, color="#3F6E80")
arrow((60, 31.5), (73, 25), rad=-0.1, color="#3F6E80")
arrow((46, 14.5), (52, 14.5), color="#3F6E80")

out_png = os.path.join(HERE, "plots", "1.png")
out_pdf = os.path.join(HERE, "plots", "1.pdf")
fig.savefig(out_png, dpi=300, bbox_inches="tight", facecolor="white")
fig.savefig(out_pdf, bbox_inches="tight", facecolor="white")
print("->", out_png)
print("->", out_pdf)
