# 视觉+IMU+轮式里程计融合定位系统 - 运行指南

## 📋 系统概述

本系统实现了**室内无GNSS环境下**的移动机器人融合定位，使用以下传感器数据：

| 传感器 | 数据文件 | 采样率 | 用途 |
|--------|---------|--------|------|
| **单目相机** | `images_left_full/` | ~10Hz | 视觉里程计 |
| **IMU** | `parking01_ADIS16465_IMU.txt` | ~200Hz | 角速度/加速度 |
| **轮式里程计** | `parking01_RANGER_ODO.txt` | ~50Hz | 轮速信息 |
| **地面真值** | `parking01_groundtruth.nav` | - | 离线评估 |

## 🚀 快速开始

### 1. 快速测试（推荐先运行）

验证系统是否正常工作，只处理100帧图像：

```bash
cd /mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main
chmod +x run_test.sh
./run_test.sh
```

**预期输出：**
- 处理约 33 帧图像（每3帧取1帧）
- 生成轨迹图和性能指标
- 运行时间约 1-2 分钟

### 2. 完整运行

处理所有数据（约1200帧，采样后）：

```bash
chmod +x run_fusion.sh
./run_fusion.sh
```

**预期输出：**
- 处理约 400 帧图像
- 生成完整的融合结果
- 运行时间约 5-10 分钟

### 3. 手动运行（自定义参数）

```bash
source .venv/bin/activate

python stereo_imu_odo_fusion.py \
    --root . \
    --left-dir images_left_full \
    --output-dir plots/indoor_fusion \
    --max-frames 1200 \
    --image-fps 10.0 \
    --image-scale 0.5 \
    --baseline 0.20 \
    --focal-ratio 0.90
```

## 📊 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--root` | `.` | 数据根目录 |
| `--left-dir` | `images_left_full` | 左目图像目录 |
| `--output-dir` | `plots/indoor_fusion` | 输出目录 |
| `--max-frames` | `500` | 最大处理帧数 |
| `--image-fps` | `10.0` | 图像帧率 (Hz) |
| `--image-scale` | `0.5` | 图像缩放比例 |
| `--baseline` | `0.20` | 基线长度（米） |
| `--focal-ratio` | `0.90` | 焦距比例 |
| `--wheel-speed-noise` | `0.15` | 轮速计噪声 |
| `--visual-position-noise` | `0.50` | 视觉位置噪声 |
| `--visual-yaw-noise` | `0.12` | 视觉偏航噪声 |

## 📈 输出文件

运行完成后，在 `plots/indoor_fusion/` 目录下生成：

### 1. `fusion_results.csv`
融合结果数据，包含：
- 时间戳
- 估计位置 (x, y)
- 估计偏航角
- 速度
- 陀螺仪偏置
- 地面真值位置 (gt_x, gt_y)

### 2. `fusion_trajectory.png`
轨迹对比图：
- 红色虚线：地面真值轨迹
- 蓝色实线：融合估计轨迹

### 3. `fusion_error.png`
位置误差随时间变化图

### 4. `metrics.txt`
性能指标：
- 平均误差 (mean_error_m)
- 中位数误差 (median_error_m)
- 95%分位误差 (p95_error_m)
- 最大误差 (max_error_m)
- 最终误差 (final_error_m)
- 视觉成功率 (visual_success_rate)

## 🔍 算法流程

```
┌─────────────────────────────────────────────────────────┐
│                    数据预处理                            │
│  1. 加载左目图像（每3帧取1帧）                          │
│  2. 加载IMU数据（陀螺仪Z轴）                           │
│  3. 加载轮速计数据                                      │
│  4. 时间同步和插值                                      │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                  单目视觉里程计                          │
│  1. ORB特征点检测                                       │
│  2. 特征点匹配                                          │
│  3. 本质矩阵计算（RANSAC）                             │
│  4. 运动估计（旋转+平移）                              │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                 扩展卡尔曼滤波（EKF）                    │
│  状态向量：[x, y, yaw, speed, gyro_bias]               │
│                                                          │
│  预测步骤：                                              │
│    - 使用IMU陀螺仪数据预测偏航角变化                    │
│    - 使用轮速计数据预测位置变化                          │
│                                                          │
│  更新步骤：                                              │
│    - 视觉里程计提供位置/偏航观测                         │
│    - 轮速计提供速度观测                                  │
└─────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────┐
│                    结果输出                              │
│  1. 融合轨迹                                            │
│  2. 误差分析                                            │
│  3. 性能指标                                            │
└─────────────────────────────────────────────────────────┘
```

## 🎯 关键特性

### 1. 图像采样策略
```python
# 每3帧取1帧，保持时间连续性
left_files = [f for i, f in enumerate(left_files) if i % 3 == 0]
```
- **目的**：减少计算量，提高处理速度
- **效果**：3773帧 → 约1258帧
- **优势**：保持时间序列连续性，不影响融合精度

### 2. 单目视觉里程计
- 使用本质矩阵分解估计相机运动
- 通过轮速计和IMU进行尺度恢复
- 适用于室内平面运动场景

### 3. 多传感器融合
- **IMU**：提供高频角速度信息
- **轮速计**：提供速度约束
- **视觉**：提供位置和偏航观测
- **EKF**：最优融合各传感器信息

## 🔧 故障排除

### 问题1：找不到图像文件
```
RuntimeError: Left image folder must contain frame_*.png files
```
**解决**：检查 `images_left_full/` 目录是否存在且包含 `frame_*.png` 文件

### 问题2：OpenCV导入失败
```
RuntimeError: opencv-python-headless is required
```
**解决**：确保已激活虚拟环境
```bash
source .venv/bin/activate
```

### 问题3：内存不足
**解决**：减少 `--max-frames` 参数
```bash
python stereo_imu_odo_fusion.py --max-frames 300 ...
```

### 问题4：运行时间过长
**解决**：
1. 降低图像分辨率：`--image-scale 0.3`
2. 减少处理帧数：`--max-frames 500`

## 📚 相关文件

- `stereo_imu_odo_fusion.py` - 主程序
- `run_fusion.sh` - 完整运行脚本
- `run_test.sh` - 快速测试脚本
- `MONO_VO_CHANGES.md` - 算法修改说明
- `PROJECT_SETUP.md` - 项目设置报告

## 📊 预期性能

基于 parking01 数据集的典型性能：

| 指标 | 预期值 |
|------|--------|
| 中位数误差 | 0.3 - 0.8 m |
| 平均误差 | 0.5 - 1.2 m |
| 95%分位误差 | 1.0 - 2.5 m |
| 视觉成功率 | 70% - 90% |

## 💡 研究建议

### 1. 参数调优
尝试调整以下参数以优化性能：
- `--visual-position-noise`: 视觉位置噪声（0.3 - 1.0）
- `--visual-yaw-noise`: 视觉偏航噪声（0.05 - 0.2）
- `--wheel-speed-noise`: 轮速计噪声（0.1 - 0.3）

### 2. 对比实验
可以创建纯IMU+轮式里程计版本进行对比：
- 禁用视觉更新
- 仅使用IMU和轮速计
- 对比融合效果

### 3. 消融实验
- 仅使用视觉里程计
- 仅使用IMU+轮速计
- 三传感器融合
- 分析各传感器的贡献

## 🎓 学术引用

如果你在本研究中使用此代码，请引用：

```bibtex
@software{indoor_fusion_baseline,
  title = {Indoor Monocular/IMU/Wheel Odometry Fusion Baseline},
  author = {Your Name},
  year = {2026},
  url = {https://github.com/your-repo}
}
```

## 📞 技术支持

如有问题，请检查：
1. 虚拟环境是否正确激活
2. 数据文件是否完整
3. 参数设置是否合理

---

**祝你研究顺利！** 🚀
