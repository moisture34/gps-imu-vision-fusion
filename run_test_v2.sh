#!/usr/bin/env bash
# Quick test of the left-camera-only + IMU + wheel-odometry fusion pipeline.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PYTHON="$SCRIPT_DIR/.venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
    echo "错误: 未找到可用的虚拟环境 Python: $PYTHON" >&2
    echo "请先创建项目虚拟环境并安装 requirements.txt。" >&2
    exit 1
fi

DATA_DIR="${DATA_DIR:-$SCRIPT_DIR/../parking01}"
if [[ ! -f "$DATA_DIR/parking01_ADIS16465_IMU.txt" || ! -f "$DATA_DIR/parking01_RANGER_ODO.txt" ]]; then
    DATA_DIR="$SCRIPT_DIR"
fi

IMAGE_DIR="${IMAGE_DIR:-$SCRIPT_DIR/stereo_images}"
OUTPUT_DIR="${OUTPUT_DIR:-plots/test_run}"
MAX_FRAMES="${MAX_FRAMES:-100}"
WHEEL_TRACK_WIDTH="${WHEEL_TRACK_WIDTH:-0.364}"
DEPTH_CHECKPOINT="${DEPTH_CHECKPOINT:-}"
VO_TRANSLATION_SOURCE="${VO_TRANSLATION_SOURCE:-wheel}"
DEPTH_ARGS=(--vo_translation_source "$VO_TRANSLATION_SOURCE")
if [[ -n "$DEPTH_CHECKPOINT" ]]; then
    DEPTH_ARGS+=(--depth_checkpoint "$DEPTH_CHECKPOINT")
fi

if [[ ! -d "$IMAGE_DIR/left" ]]; then
    echo "错误: 未找到左目图像目录 $IMAGE_DIR/left" >&2
    exit 1
fi

LEFT_COUNT="$(find "$IMAGE_DIR/left" -maxdepth 1 -type f -name 'frame_*.png' | wc -l)"
if (( LEFT_COUNT == 0 )); then
    echo "错误: 左目图像目录必须至少有一张 frame_*.png 图像" >&2
    exit 1
fi

for data_file in parking01_ADIS16465_IMU.txt parking01_RANGER_ODO.txt; do
    if [[ ! -f "$DATA_DIR/$data_file" ]]; then
        echo "错误: 未找到数据文件 $DATA_DIR/$data_file" >&2
        exit 1
    fi
done
if [[ ! -f "$SCRIPT_DIR/calibration.yaml" ]]; then
    echo "错误: 未找到相机标定文件 $SCRIPT_DIR/calibration.yaml" >&2
    exit 1
fi

echo "=================================================="
echo "快速测试 - 左目单目视觉 + IMU + 轮速计融合"
echo "=================================================="
echo "左目图像: $LEFT_COUNT 张"
echo "数据目录: $DATA_DIR"
echo "处理帧数: $MAX_FRAMES"
echo "轮距: $WHEEL_TRACK_WIDTH m"
echo "输出目录: $OUTPUT_DIR"
echo ""

"$PYTHON" "$SCRIPT_DIR/stereo_imu_odo_fusion_v2.py" \
    --calibration "$SCRIPT_DIR/calibration.yaml" \
    --left_dir "$IMAGE_DIR/left" \
    --imu_file "$DATA_DIR/parking01_ADIS16465_IMU.txt" \
    --odo_file "$DATA_DIR/parking01_RANGER_ODO.txt" \
    --output_dir "$OUTPUT_DIR" \
    --timestamps_file "$IMAGE_DIR/left/timestamps.txt" \
    --image_scale 0.5 \
    --max_frames "$MAX_FRAMES" \
    --wheel_track_width "$WHEEL_TRACK_WIDTH" \
    --init_method imu_alignment \
    --apply_gyro_bias \
    --imu_prop_hz 50 \
    --visual_position_noise 3.5 \
    "${DEPTH_ARGS[@]}"

echo ""
echo "IMU静止对准部署模式完成（运行不读取任何真值，已应用静止陀螺零偏修正）。"
echo "注意：本快速测试只跑前 $MAX_FRAMES 帧。短轨迹上轮速尚未漂移，视觉收益不明显，"
echo "      全轨迹精度请看 run_full_trajectory.sh 与 README 的 2×2 消融表。"
echo "结果: $OUTPUT_DIR/fusion_results.csv"
echo "轨迹图: $OUTPUT_DIR/trajectory.png"
