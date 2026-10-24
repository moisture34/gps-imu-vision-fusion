# 视觉、IMU 与轮速计三源融合定位系统

基于扩展卡尔曼滤波（EKF）的移动机器人融合定位研究项目。项目面向室内无 GNSS 场景，结合视觉里程计、惯性测量单元（IMU）与轮速计，提供数据处理、轨迹融合及离线评估脚本。

## 项目功能

- 基于双目图像与特征匹配的视觉里程计及运动估计。
- 融合 IMU 航向/角速度与轮速信息的平面 EKF 定位。
- 可选的陀螺零偏修正、车道线航向约束和单目深度平移估计实验。
- 支持消融实验、轨迹与视觉运动评估，并生成对比图表。
- 提供相机标定参数、示例传感器数据及中文项目说明材料。

## 项目文件

| 文件或目录 | 说明 |
| --- | --- |
| `stereo_imu_odo_fusion.py`、`stereo_imu_odo_fusion_v2.py` | 视觉、IMU、轮速计融合程序 |
| `wheel_odometry.py`、`heading_lever.py`、`lane_heading.py` | 轮速、航向及车道线相关模块 |
| `evaluate_trajectory.py`、`evaluate_vo_motion.py` | 轨迹与视觉运动离线评估 |
| `calibration.yaml` | 相机及传感器标定参数 |
| `parking01_*.txt`、`parking01_*.nav`、`parking01_trajectory.csv` | parking01 示例传感器与参考轨迹数据 |
| `depth_anything_v2_metric_hypersim_vits.pth` | 深度估计模型权重 |
| `README_V2.md`、`QUICK_START.md`、`RUN_GUIDE.md` | 详细中文说明与运行指南 |
| `说明书插图/`、`*.docx` | 项目插图、作品说明书及申报材料 |

## 环境准备

- Python 3.8 或更高版本。
- 安装 Python 依赖：

```bash
pip install -r requirements.txt
```

如需从 ROS bag 提取图像，按 `requirements.txt` 安装 `rosbags`，并参考 `extract_stereo_from_bag.py`。部分深度估计功能需要 PyTorch 及兼容的计算设备。

## 快速运行

在项目目录中执行快速测试脚本：

```bash
bash run_test_v2.sh
```

运行全轨迹流程：

```bash
bash run_full_trajectory.sh
```

也可以直接调用融合程序。下例中的图像目录、时间戳文件及输出目录需按本地数据调整：

```bash
python stereo_imu_odo_fusion_v2.py \
  --calibration calibration.yaml \
  --left_dir stereo_images/left \
  --timestamps_file stereo_images/left/timestamps.txt \
  --imu_file parking01_ADIS16465_IMU.txt \
  --odo_file parking01_RANGER_ODO.txt \
  --apply_gyro_bias \
  --output_dir plots/experiment
```

Windows 用户可使用 Python 环境中对应的解释器运行 `.py` 程序；`.sh` 脚本需在 Bash 环境中执行。完整参数、数据准备方法和实验说明请参阅 [快速开始](QUICK_START.md)、[运行指南](RUN_GUIDE.md) 与 [详细项目说明](README_V2.md)。

## 数据、模型与使用说明

仓库包含传感器记录、参考轨迹、模型权重及项目文档。使用或再分发这些数据和权重前，请自行核实其来源、许可及适用范围。实验结果受相机时间戳、标定参数、轮距和数据质量影响；请勿将示例配置或离线评估结果直接视为适用于其他设备的精度保证。

## 依赖

主要依赖包括 NumPy、OpenCV、Matplotlib、PyYAML、PyTorch、Kornia 和 rosbags。具体版本范围见 [`requirements.txt`](requirements.txt)。
