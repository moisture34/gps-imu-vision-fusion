# 虚拟环境激活说明

## 环境信息
- **Python 版本**: 3.8.10
- **虚拟环境路径**: `.venv/`
- **操作系统**: Linux

## 已安装的依赖包
- numpy 1.24.4
- matplotlib 3.7.5
- opencv-python-headless 5.0.0
- utm 0.8.1

## 激活虚拟环境

### Linux / macOS
```bash
source .venv/bin/activate
```

###  deactivate（退出虚拟环境）
```bash
deactivate
```

## 运行项目脚本

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

## 验证安装
```bash
source .venv/bin/activate
python -c "import numpy, matplotlib, cv2, utm; print('所有依赖安装成功！')"
```

## 注意事项
1. 原有的 Windows 虚拟环境（Python 3.12.9）已删除，因为无法在 Linux 上使用
2. 新虚拟环境使用 Python 3.8.10，与当前系统兼容
3. 如果需要使用双目视觉融合脚本，请确保 `images_right` 目录存在且包含右目图像
