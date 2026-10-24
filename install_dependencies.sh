#!/bin/bash
# 安装所有依赖

echo "=================================================="
echo "安装双目视觉融合定位系统依赖"
echo "=================================================="
echo ""

# 激活虚拟环境
source .venv/bin/activate

echo "正在安装依赖包..."
echo ""

# 安装所有依赖
pip install -r requirements.txt

echo ""
echo "=================================================="
echo "依赖安装完成！"
echo "=================================================="
echo ""

# 验证安装
python -c "
import sys
print('验证安装:')
print()

packages = {
    'numpy': 'numpy',
    'cv2': 'opencv-python-headless',
    'matplotlib': 'matplotlib',
    'yaml': 'pyyaml',
    'rosbags': 'rosbags'
}

all_ok = True
for module, package in packages.items():
    try:
        mod = __import__(module)
        version = getattr(mod, '__version__', 'unknown')
        print(f'  ✓ {package} ({version})')
    except ImportError:
        print(f'  ✗ {package} (未安装)')
        all_ok = False

print()
if all_ok:
    print('所有依赖安装成功！')
else:
    print('部分依赖未安装，请检查错误信息')
"
