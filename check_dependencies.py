#!/mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main/.venv/bin/python3
"""
测试所有依赖是否正确安装
"""

import sys

print("=" * 60)
print("依赖检查")
print("=" * 60)
print()

# 检查 Python 版本
print(f"Python版本: {sys.version}")
print(f"Python路径: {sys.executable}")
print()

# 检查所有依赖
packages = {
    'numpy': 'numpy',
    'cv2': 'opencv-python-headless',
    'matplotlib': 'matplotlib',
    'yaml': 'pyyaml',
    'rosbags': 'rosbags'
}

print("依赖包检查:")
all_ok = True
for module, package in packages.items():
    try:
        mod = __import__(module)
        version = getattr(mod, '__version__', 'unknown')
        print(f'  ✓ {package:30s} ({version})')
    except ImportError as e:
        print(f'  ✗ {package:30s} (未安装: {e})')
        all_ok = False

print()
print("=" * 60)
if all_ok:
    print("✓ 所有依赖检查通过！")
    print("=" * 60)
    sys.exit(0)
else:
    print("✗ 部分依赖未安装")
    print("=" * 60)
    print()
    print("请运行以下命令安装依赖:")
    print("  source .venv/bin/activate")
    print("  pip install -r requirements.txt")
    sys.exit(1)
