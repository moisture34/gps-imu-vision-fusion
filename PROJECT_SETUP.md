# 项目设置完成报告

## ✅ 已完成的任务

### 1. 虚拟环境创建
- **位置**: `.venv/`
- **Python 版本**: 3.8.10 (Linux)
- **状态**: ✅ 已成功创建并安装所有依赖

#### 已安装的依赖包
```
numpy==1.24.4
matplotlib==3.7.5
opencv-python-headless==5.0.0
utm==0.8.1
```

#### 激活虚拟环境
```bash
source .venv/bin/activate
```

### 2. 左目图像采样逻辑修改
- **文件**: `stereo_imu_odo_fusion.py`
- **修改位置**: 第 252-255 行
- **状态**: ✅ 已添加每3帧取1帧的逻辑

#### 修改内容
```python
# 每3帧取1帧，保持连续的
left_files = [f for i, f in enumerate(left_files) if i % 3 == 0]
right_files = [f for i, f in enumerate(right_files) if i % 3 == 0]
count = len(left_files)
```

#### 说明
- 对左目和右目图像同时进行每3帧取1帧的采样
- 保持左右目图像的同步（双目视觉需要同步）
- 采样后帧数约为原来的 1/3
- 例如：原始 3773 帧 → 采样后约 1258 帧

## 📋 运行项目

### 1. GPS-IMU 融合（EKF）
```bash
source .venv/bin/activate
python imu_gps_kf_fusion.py
```

### 2. 双目视觉 + IMU + 轮速计融合
```bash
source .venv/bin/activate
python stereo_imu_odo_fusion.py --root . --left-dir images_left_full --right-dir images_right
```

**注意**: 运行双目视觉融合脚本前，请确保 `images_right` 目录存在且包含右目图像。

## 🔍 验证安装

```bash
source .venv/bin/activate
python -c "import numpy, matplotlib, cv2, utm; print('✓ 所有依赖安装成功！')"
```

## 📝 重要说明

1. **虚拟环境兼容性**: 原有的 Windows 虚拟环境（Python 3.12.9）已删除，因为无法在 Linux 上使用。新虚拟环境使用 Python 3.8.10，与当前系统完全兼容。

2. **图像采样逻辑**: 代码已修改为每3帧取1帧，这会显著减少处理的图像数量，加快运行速度，同时保持时间序列的连续性。

3. **双目图像同步**: 左右目图像使用相同的采样逻辑，确保双目视觉的同步性。

4. **数据文件**: 项目包含以下数据文件：
   - `parking01_ADIS16465_IMU.txt` - IMU 数据
   - `parking01_MID360_IMU.txt` - MID360 IMU 数据
   - `parking01_RANGER_ODO.txt` - 轮速计数据
   - `parking01_groundtruth.nav` - 地面真值数据
   - `parking01_trajectory.csv` - 轨迹数据
   - `images_left_full/` - 左目图像（3773 帧）

## 🎯 预期效果

- 采样后，双目视觉融合的处理速度将提升约 3 倍
- 由于帧数减少，内存占用也会显著降低
- 保持时间序列的连续性，不会影响融合精度
