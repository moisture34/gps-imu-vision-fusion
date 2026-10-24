#!/usr/bin/env bash
# 全轨迹 (17分钟, ~3772帧) 运行编排 —— VO/IMU消融及DA米制深度对照：
#   视觉更新 (VO) 开/关 × 陀螺静止零偏修正 开/关，并对比轮速尺度与DA深度PnP
#
#   0) 需要时从 bag 提取图像（含真实时间戳）
#   1) 四组融合（均不读取真值，IMU静止对准初始化）
#   2) 逐组与真值评估（2D刚体对齐）
#   3) 逐帧VO位移误差 + 融合轨迹对比图
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PYTHON="$SCRIPT_DIR/.venv/bin/python3"
DATA_DIR="${DATA_DIR:-$SCRIPT_DIR/../parking01}"
IMAGE_DIR="${IMAGE_DIR:-$SCRIPT_DIR/stereo_images}"
BAG_FILE="${BAG_FILE:-$SCRIPT_DIR/parking01.bag}"
GT="$DATA_DIR/parking01_groundtruth.nav"
OUT_ROOT="${OUT_ROOT:-plots/ablation}"
LEFT_COUNT="$(find "$IMAGE_DIR/left" -maxdepth 1 -type f -name 'frame_*.png' | wc -l)"

# 步骤0: 若尚未提取（或缺少真实时间戳），先从 bag 单遍提取
if [ "$LEFT_COUNT" -lt 100 ] || [ ! -f "$IMAGE_DIR/left/timestamps.txt" ]; then
    if [ ! -f "$BAG_FILE" ]; then
        echo "错误: 需要图像提取但找不到 bag: $BAG_FILE" >&2
        exit 1
    fi
    echo "=== [0] 从 bag 提取图像（每3帧1帧，仅左目，含真实时间戳）==="
    "$PYTHON" "$SCRIPT_DIR/extract_stereo_from_bag.py" "$BAG_FILE" "$IMAGE_DIR" 3 --left-only
    LEFT_COUNT="$(find "$IMAGE_DIR/left" -maxdepth 1 -type f -name 'frame_*.png' | wc -l)"
fi

if [ "$LEFT_COUNT" -lt 100 ]; then
    echo "错误: $IMAGE_DIR/left 只有 $LEFT_COUNT 帧，请先运行图像提取。" >&2
    exit 1
fi
echo "可用左目帧: $LEFT_COUNT"

COMMON=(--calibration "$SCRIPT_DIR/calibration.yaml"
        --left_dir "$IMAGE_DIR/left"
        --imu_file "$DATA_DIR/parking01_ADIS16465_IMU.txt"
        --odo_file "$DATA_DIR/parking01_RANGER_ODO.txt"
        --timestamps_file "$IMAGE_DIR/left/timestamps.txt"
        --image_scale 0.5
        --max_frames "$LEFT_COUNT"
        --wheel_track_width 0.364
        --init_method imu_alignment)

run_one() {  # run_one <名称> <视觉噪声> <是否应用零偏:yes/no> [额外参数...]
    local name="$1" vnoise="$2" bias="$3"; shift 3
    local extra=()
    [ "$bias" = "yes" ] && extra+=(--apply_gyro_bias)
    echo ""
    echo "=== 融合 [$name]  视觉噪声=$vnoise  零偏修正=$bias  附加: $* ==="
    "$PYTHON" "$SCRIPT_DIR/stereo_imu_odo_fusion_v2.py" "${COMMON[@]}" \
        --visual_position_noise "$vnoise" "${extra[@]}" "$@" \
        --output_dir "$OUT_ROOT/$name" \
        --groundtruth "$GT"
    "$PYTHON" "$SCRIPT_DIR/evaluate_trajectory.py" \
        --estimated "$OUT_ROOT/$name/fusion_results.csv" \
        --groundtruth "$GT" --output_dir "$OUT_ROOT/$name" --align
}

run_one novo_nobias  1e6 no
run_one novo_bias    1e6 yes
run_one vo_nobias    3.5 no
run_one vo_bias      3.5 yes
# 对照组：放开 yaw 保护，让位置更新经协方差耦合反算偏航（长轨迹会明显恶化）
run_one vo_bias_yawcoupled 3.5 yes --visual_update_yaw

# ── 扩展对照：README「逻辑审查发现」各表的出处，额外约 45 分钟 ─────────────────
# 用法: WITH_CONTROLS=1 ./run_full_trajectory.sh
CTRL="$SCRIPT_DIR/plots/controls"
run_probe() {   # run_probe <相对路径> <融合附加参数...>
    local name="$1"; shift
    echo ""; echo "=== 对照 [$name] $* ==="
    "$PYTHON" "$SCRIPT_DIR/stereo_imu_odo_fusion_v2.py" "${COMMON[@]}" \
        --visual_position_noise 3.5 --apply_gyro_bias "$@" \
        --output_dir "$CTRL/$name" --groundtruth "$GT"
    "$PYTHON" "$SCRIPT_DIR/evaluate_trajectory.py" \
        --estimated "$CTRL/$name/fusion_results.csv" \
        --groundtruth "$GT" --output_dir "$CTRL/$name" --align
}
if [ "${WITH_CONTROLS:-0}" = "1" ]; then
    # ② 增益归因：更新照做但抹掉图像信息 → 分离"图像信息"与"协方差整形"
    run_probe vo_probe_no_lateral --vo_probe no_lateral
    # ⑤ 视觉噪声重扫（结论：3.5 为最优，越小越差）
    for R in 1.0 0.35 0.1; do
        run_probe "visual_noise_sweep/R$R" --visual_position_noise "$R"
    done
    # ④ 定标口径 A/B；以及初始航向来源对照（绝对定位缺口的量化）
    run_probe repro_and_init/vo_scale_integral --vo_scale_integral
    run_probe repro_and_init/gt_init --init_method groundtruth
    # 深度支路三变体：仅边缘筛除 / 米制 PnP 定标 / 米制 PnP + 动态尺度标定
    DEPTH_CKPT="$SCRIPT_DIR/depth_anything_v2_metric_hypersim_vits.pth"
    run_probe depth_edge_filter/vo_bias_depth  --depth_checkpoint "$DEPTH_CKPT"
    run_probe depth_metric/vo_bias_depth       --depth_checkpoint "$DEPTH_CKPT" \
                                                --vo_translation_source depth
    run_probe depth_dynamic/vo_bias_depth_dynamic --depth_checkpoint "$DEPTH_CKPT" \
                                                --vo_translation_source depth_dynamic
fi

echo ""
echo "=== VO 帧间平移距离评估（与真值对比，不参与融合）==="
for csv in "$OUT_ROOT"/*/vo_motion_results.csv "$CTRL"/*/vo_motion_results.csv \
           "$CTRL"/*/*/vo_motion_results.csv; do
    [ -f "$csv" ] || continue
    "$PYTHON" "$SCRIPT_DIR/evaluate_vo_motion.py" \
        --vo "$csv" --groundtruth "$GT" --output_dir "$(dirname "$csv")"
done

echo ""
echo "=== 轨迹对比图 ==="
"$PYTHON" "$SCRIPT_DIR/plot_vo_comparison.py" \
    --groundtruth "$GT" \
    --prefix main \
    --results \
        "novo_nobias=$OUT_ROOT/novo_nobias/fusion_results.csv" \
        "vo_nobias=$OUT_ROOT/vo_nobias/fusion_results.csv" \
        "novo_bias=$OUT_ROOT/novo_bias/fusion_results.csv" \
        "vo_bias=$OUT_ROOT/vo_bias/fusion_results.csv" \
    --labels \
        "①无视觉 无零偏(纯航位推算 8.24m)" \
        "②有视觉 无零偏(7.66m)" \
        "③无视觉 +静止零偏修正(1.17m)" \
        "④有视觉 +静止零偏修正(最佳 0.89m)" \
    --output_dir "$OUT_ROOT/comparison"

# 扩展对照存在时，另出一张"有视觉 vs 无视觉 vs DA 三种用法"的对比图
if [ "${WITH_CONTROLS:-0}" = "1" ] && [ -f "$CTRL/depth_metric/vo_bias_depth/fusion_results.csv" ]; then
    "$PYTHON" "$SCRIPT_DIR/plot_vo_comparison.py" \
        --groundtruth "$GT" \
        --prefix da \
        --results \
            "no_vision=$OUT_ROOT/novo_bias/fusion_results.csv" \
            "vo_wheel=$OUT_ROOT/vo_bias/fusion_results.csv" \
            "edge=$CTRL/depth_edge_filter/vo_bias_depth/fusion_results.csv" \
            "metric=$CTRL/depth_metric/vo_bias_depth/fusion_results.csv" \
            "dynamic=$CTRL/depth_dynamic/vo_bias_depth_dynamic/fusion_results.csv" \
        --labels \
            "无视觉(仅IMU+轮速 1.17m)" \
            "VO基线: 轮速定标平移(0.89m)" \
            "VO+DA深度边缘筛除(0.88m)" \
            "VO+DA米制深度PnP定标(3.52m)" \
            "VO+DA米制PnP+动态尺度(0.99m)" \
        --output_dir "$OUT_ROOT/comparison"
fi

echo ""
echo "=== 报告总览图 ==="
"$PYTHON" "$SCRIPT_DIR/make_report_figures.py" \
    --groundtruth "$GT" --run "$OUT_ROOT/vo_bias/fusion_results.csv" \
    --output_dir "$OUT_ROOT"

echo ""
echo "全部完成。主消融: $OUT_ROOT   扩展对照: $CTRL"
