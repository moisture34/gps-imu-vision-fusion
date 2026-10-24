#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_lane_probe.py —— 汇总 lane_probe 的基线与姿态灵敏度扫描，出结论表与报告图。

用法：先跑 lane_probe.py 的各组扫描（--out plots/lane_probe/<tag>），再跑本脚本。
    .venv/bin/python3 analyze_lane_probe.py

输出 plots/lane_probe/final_report.txt 与 lane_probe_summary.png
"""
import argparse
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

COLS = dict(t=0, speed=1, yaw=2, yr=3, roll=4, pitch=5, vh=6, nseg=7,
            a1=8, spread=9, frac=10, a2=11, orth=12, c90=13)


def load(d):
    p = os.path.join(d, "probe_results.csv")
    if not os.path.exists(p):
        return None
    a = np.loadtxt(p, delimiter=",", skiprows=1, usecols=range(1, 15))
    return a if a.size else None


def resid(a, gate=True, max_yr=6.0, frac_min=0.80, spread_max=0.60):
    """返回 (直行帧残差数组, 门限掩码, 全部直行掩码)。残差 = wrap90(c90 − 全局中位数)。"""
    yr, c90, f, s = a[:, COLS["yr"]], a[:, COLS["c90"]], a[:, COLS["frac"]], a[:, COLS["spread"]]
    straight = np.abs(yr) < max_yr
    g = straight & (f >= frac_min) & (s <= spread_max) if gate else straight
    if g.sum() < 5:
        return np.array([]), g, straight
    ref = np.median(c90[straight]) if straight.sum() >= 5 else np.median(c90)
    return np.mod(c90[g] - ref + 45.0, 90.0) - 45.0, g, straight


def stats(r):
    if len(r) < 5:
        return dict(n=len(r))
    return dict(n=len(r), mean=float(np.mean(r)), std=float(np.std(r)),
                p50=float(np.percentile(np.abs(r), 50)),
                p68=float(np.percentile(np.abs(r), 68)),
                p95=float(np.percentile(np.abs(r), 95)),
                mx=float(np.abs(r).max()),
                out=float(100.0 * (np.abs(r) > 3.0).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=os.path.join(HERE, "plots", "lane_probe"))
    ap.add_argument("--tags", default="base,gtclean,online,fixed,p05,r05,p10,r10")
    ap.add_argument("--stride-s", type=float, default=1.8,
                    help="stride=6 帧 × 0.3 s/帧 的采样周期（秒）")
    args = ap.parse_args()

    lines = ["lane_probe 汇总结论", "=" * 96]
    hdr = ("%-7s %-22s %5s %6s %6s %6s %6s %6s %6s"
           % ("组", "姿态来源/扰动", "n", "通过率", "P50°", "P68°", "P95°", "最大°", ">3°%"))
    lines.append(hdr)
    lines.append("-" * 96)
    data = {}
    for tag in args.tags.split(","):
        d = os.path.join(args.root, tag)
        a = load(d)
        if a is None:
            lines.append("%-7s (未跑)" % tag)
            continue
        r_nogate, _, straight = resid(a, gate=False)
        r_gate, g, _ = resid(a, gate=True)
        s1, s2 = stats(r_nogate), stats(r_gate)
        note = dict(base="GT 原始姿态(含尖峰)", gtclean="GT 0.5s中值(评估上界)",
                    online="在线互补滤波(部署口径)",
                    fixed="固定=起始对准",
                    p05="GT + pitch 扰动0.5°", r05="GT + roll 扰动0.5°",
                    p10="GT + pitch 扰动1.0°", r10="GT + roll 扰动1.0°").get(tag, tag)
        lines.append("%-7s %-22s %5d %5.0f%% %6.2f %6.2f %6.2f %6.2f %6.1f"
                     % (tag, note, s2.get("n", 0),
                        100.0 * g.sum() / max(len(a), 1),
                        s2.get("p50", np.nan), s2.get("p68", np.nan),
                        s2.get("p95", np.nan), s2.get("mx", np.nan),
                        s2.get("out", np.nan)))
        lines.append("%-7s %-22s %5d %5s %6.2f %6.2f %6s %6s %6.1f  <- 不加门限"
                     % ("", "", s1.get("n", 0), "", s1.get("p50", np.nan),
                        s1.get("p68", np.nan), "", "", s1.get("out", np.nan)))
        data[tag] = (a, r_gate, g, straight)

    if "base" in data:
        a, r, g, straight = data["base"]
        lines.append("")
        lines.append("base 组细节（GT 姿态，门限内 %d 帧）：" % g.sum())
        lines.append("  残差 均值 %+.3f°  标准差 %.3f°  P95 %.2f°  最大 %.2f°"
                     % (np.mean(r), np.std(r), np.percentile(np.abs(r), 95), np.abs(r).max()))
        ref = np.median(a[straight][:, COLS["c90"]]) if straight.sum() else np.median(a[:, COLS["c90"]])
        lines.append("  车道网方位常数（一次性场地标定值，本序列实测）= %+.3f°" % ref)
        t = a[:, COLS["t"]]
        t0 = t[0]
        lines.append("  分 120 s 时段中位数（检验常数是否漂移）：")
        seg = []
        for k in range(int((t[-1] - t0) / 120) + 1):
            m = g & (t >= t0 + 120 * k) & (t < t0 + 120 * (k + 1))
            if m.sum() >= 8:
                rr = np.mod(a[m][:, COLS["c90"]] - ref + 45, 90) - 45
                seg.append((k * 120, m.sum(), float(np.median(rr))))
                lines.append("    %4d-%4ds n=%3d 中位 %+5.2f°" % (120 * k, 120 * (k + 1), m.sum(), np.median(rr)))
        spread_seg = max(x[2] for x in seg) - min(x[2] for x in seg) if seg else np.nan
        lines.append("  => 时段间最大离散 %.2f°（<0.3° 即证明单一常数成立）" % spread_seg)
        lines.append("  观测可用率：门限内 %d/%d = %.0f%%，等效每 %.1f s 一次有效航向观测"
                     % (g.sum(), len(a), 100.0 * g.mean(), args.stride_s / max(g.mean(), 1e-9)))
        lines.append("  转弯帧 |yaw rate|>=6°/s 占 %.0f%%（这些帧本就不该用车道航向）"
                     % (100.0 * (1 - straight.mean())))

    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(args.root, "final_report.txt"), "w", encoding="utf-8") as f:
        f.write(txt + "\n")

    # ------------------------------- 报告图 -------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    # 显式注册中文字体，避免 Matplotlib 字体缓存或跨环境回退造成方框和乱码。
    font_candidates = [
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\simhei.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ]
    font_name = "DejaVu Sans"
    for font_path in font_candidates:
        if os.path.exists(font_path):
            font_manager.fontManager.addfont(font_path)
            font_name = font_manager.FontProperties(fname=font_path).get_name()
            break
    plt.rcParams["font.family"] = "sans-serif"
    plt.rcParams["font.sans-serif"] = [font_name, "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["figure.dpi"] = 150
    plt.rcParams["savefig.dpi"] = 300
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.3))
    if "base" in data:
        a, r, g, straight = data["base"]
        t = a[:, COLS["t"]] - a[0, COLS["t"]]
        ax[0].plot(t[~g & straight], np.mod(a[~g & straight][:, COLS["c90"]] - np.median(a[straight][:, COLS["c90"]]) + 45, 90) - 45,
                   "x", ms=5, color="0.6", label="被门限剔除")
        ax[0].plot(t[g], r, ".", ms=4, color="tab:blue", label="门限内 (n=%d)" % g.sum())
        ax[0].axhline(0, lw=.6, color="k")
        ax[0].set_ylim(-4, 4)
        ax[0].set_xlabel("t (s)")
        ax[0].set_ylabel("航向一致性残差 (°)")
        ax[0].set_title("α+yaw 残差 vs 真值（GT姿态）\n标准差 %.2f°  P95 %.2f°" % (np.std(r), np.percentile(np.abs(r), 95)))
        ax[0].legend(fontsize=8)
    if "fixed" in data:
        a, r, g, straight = data["fixed"]
        t = a[:, COLS["t"]] - a[0, COLS["t"]]
        rr = np.mod(a[g][:, COLS["c90"]] - np.median(a[straight][:, COLS["c90"]]) + 45, 90) - 45 if g.sum() > 5 else r
        ax[1].plot(t[g], rr, ".", ms=4, color="tab:red", label="固定姿态(可部署) n=%d" % g.sum())
        if "base" in data:
            a2, r2, g2, _ = data["base"]
            ax[1].plot(a2[g2][:, 0] - a2[0, 0], r2, ".", ms=3, color="tab:blue", alpha=.5, label="GT姿态(上界)")
        ax[1].set_ylim(-4, 4)
        ax[1].set_xlabel("t (s)")
        ax[1].set_title("姿态来源决定精度\n部署需在线跟踪 roll/pitch")
        ax[1].legend(fontsize=8)
    tags = [k for k in ("base", "p05", "p10", "r05", "r10") if k in data]
    if tags:
        p68 = [stats(resid(data[k][0], True)[0])["p68"] for k in tags]
        p95 = [stats(resid(data[k][0], True)[0])["p95"] for k in tags]
        x = np.arange(len(tags))
        ax[2].bar(x - .2, p68, .38, label="P68")
        ax[2].bar(x + .2, p95, .38, label="P95")
        ax[2].set_xticks(x)
        ax[2].set_xticklabels(tags)
        ax[2].set_ylabel("残差 (°)")
        ax[2].set_title("姿态灵敏度\nroll 几乎不影响，pitch 约 0.4°/°")
        ax[2].legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(args.root, "lane_probe_summary.png")
    fig.savefig(out, dpi=300, bbox_inches="tight", facecolor="white")
    print("\n-> %s\n-> %s" % (out, os.path.join(args.root, "final_report.txt")))


if __name__ == "__main__":
    main()
