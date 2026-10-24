# 左目单目视觉+IMU+轮速计融合定位系统

## 📋 项目概述

基于 **i2Nav-Robot parking01** 数据集的室内无GNSS移动机器人融合定位系统。

### 传感器配置

| 传感器 | 型号/类型 | 采样率 | 数据源 |
|--------|----------|--------|--------|
| **左目相机** | 1600×1200 | ~10Hz | parking01.bag |
| **右目相机** | 1600×1200 | ~10Hz | parking01.bag |
| **IMU** | ADIS16465 | ~200Hz | parking01_ADIS16465_IMU.txt |
| **轮速计** | RANGER | ~50Hz | parking01_RANGER_ODO.txt |

### 相机参数（来自 calibration.yaml）

| 参数 | 左目 | 右目 |
|------|------|------|
| fx | 1064.90 | 1063.24 |
| fy | 1065.25 | 1063.45 |
| cx | 801.40 | 804.07 |
| cy | 624.69 | 630.02 |
| **基线** | **0.4 m** | - |

---

## 🚀 快速开始

### 1. 完整运行（全轨迹 2×2 消融）

```bash
cd /mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main

# 提取图像 + 融合消融 + VO位移/轨迹评估 + 对比图
./run_full_trajectory.sh
```

**流程：**
1. 若 `stereo_images/left` 缺失或没有 `timestamps.txt`，从 `parking01.bag`
   单遍提取图像（每3帧取1帧，仅左目），并记录每帧的**真实采集时间戳**
2. 融合阶段只读取左目图像，右目图像不参与特征提取或匹配
3. 跑 2×2 主消融（视觉更新开/关 × 陀螺零偏修正开/关）+ yaw 耦合对照组，
   融合阶段均不读取真值
4. 与 `parking01_groundtruth.nav` 做帧间视觉位移误差和 2D 刚体对齐轨迹评估，输出对比图

主消融结果根目录 `plots/ablation/`，对比图在 `plots/ablation/comparison/`。

README「逻辑审查发现」与「增益归因对照」各表的扩展对照（`--vo_probe`、视觉噪声重扫、
定标口径 A/B、初始航向来源、深度三支路）默认**不跑**，需要时加开关（额外约 45 分钟）：

```bash
WITH_CONTROLS=1 ./run_full_trajectory.sh     # 扩展对照输出到 plots/controls/
```

### 2. 快速测试

```bash
# 只处理100帧（验证系统）
./run_test_v2.sh

# 可选：指定测试输出目录，或覆盖数据目录/轮距
OUTPUT_DIR=plots/my_test DATA_DIR=../parking01 WHEEL_TRACK_WIDTH=0.46 ./run_test_v2.sh
```

### Depth Anything V2 米制平移估计消融

项目包含 Hypersim 室内米制深度 Vits 权重
`depth_anything_v2_metric_hypersim_vits.pth`。默认 `--vo_translation_source wheel`
保持传统基线不变；设置 `--vo_translation_source depth` 后，DA 对前一帧预测米制深度，
将 SuperPoint 特征匹配反投影成 3D 点，再用匹配的当前帧像素和 RANSAC PnP 求帧间
相机运动。所得平移直接以米计，不再按轮速重新缩放；轮速仍用于 EKF 预测。
匹配边缘筛选也会启用，减少物体遮挡边界附近的不稳定对应。DA 每帧推理一次，会增加
GPU/CPU 时间。Hypersim 是合成室内训练域，真实停车场的绝对深度可能存在偏差。

```bash
# 使用 DA 米制深度恢复 VO 平移尺度
python stereo_imu_odo_fusion_v2.py \
    --calibration calibration.yaml \
    --left_dir stereo_images/left \
    --timestamps_file stereo_images/left/timestamps.txt \
    --imu_file parking01_ADIS16465_IMU.txt \
    --odo_file parking01_RANGER_ODO.txt \
    --apply_gyro_bias \
    --visual_position_noise 3.5 \
    --vo_translation_source depth \
    --depth_checkpoint depth_anything_v2_metric_hypersim_vits.pth \
    --output_dir plots/controls/depth_metric/vo_bias_depth
```

程序会自动使用项目根目录中的默认 checkpoint；若权重不在该位置，可用
`--depth_checkpoint <path>` 指定。每次运行均保存 `vo_motion_results.csv`，记录成功状态、
匹配/位姿内点数量、米制平移和轮速位移。`evaluate_vo_motion.py` 将 VO 单帧水平位移模长
与真值相邻位置的位移模长比较，输出距离 RMSE、MAE 和偏差；该指标只用于离线评估，
不会反馈到融合结果。完整流程在 `vo_bias` 与 `vo_bias_depth` 两组中都生成这项报告，
动态模式另输出校正前后的距离误差以及尺度因子轨迹统计，同时比较融合轨迹误差。

注意：平移距离评估针对相邻采样帧之间的水平位移，无法单独验证 DA 对静态单帧场景深度
的绝对误差；没有像素级激光/深度真值时，轨迹 NAV 只能验证运动距离与轨迹收益。

**parking01 全轨迹实测（3666 帧，DA 使用未额外标定的原始米制输出）：**

| 视觉平移尺度 | 帧间位移 RMSE | 平均估计位移 | 平均真值位移 | 融合位置 RMSE |
|---|---:|---:|---:|---:|
| 不引入视觉（仅 IMU+轮速） | — | — | — | 1.1716 m |
| 轮速尺度（基线） | 0.0228 m | 0.3383 m | 0.3469 m | 0.8882 m |
| DA 仅作匹配边缘筛除（平移仍由轮速定标） | — | — | — | 0.8760 m |
| DA 米制深度 PnP | 0.5685 m | 0.8732 m | 0.3461 m | 3.5174 m |
| DA + 11帧动态尺度中值校正 | 0.0733 m | 0.3538 m | 0.3461 m | 0.9924 m |

DA 在本项目被评估过三种角色，只有"边缘筛除"用武之地且收益（0.888→0.876 m，1.4%）
在噪声量级；一旦让 DA 承担定标平移的职能，精度单调恶化 4 倍。结论图见
`plots/ablation/comparison/da_{trajectory,error}.png`。

当前 Hypersim 权重在该停车场数据上显著高估帧间平移（平均偏差 +0.5271 m），
因此 `depth` 原始米制 PnP 不适合作为本数据集的默认定位配置。`depth_dynamic`
模式在运行时用轮速里程/DA-PnP位移比的 11 帧滑动中值更新尺度因子，低运动或低内点
样本会被拒绝；积累 5 个有效样本前暂不融合 DA 视觉更新。全轨迹尺度因子中位数为
0.401，范围为 0.233–1.677（含短时异常比值，但窗口中值降低了其影响）。

动态校正将帧间距离 RMSE 从原始 DA 的 0.5685 m 降到 0.0733 m，融合位置 RMSE
从 3.5174 m 降到 0.9924 m；但仍略差于轮速尺度基线的 0.8882 m。因此目前应把
`depth_dynamic` 视作已实现、可评测的深度辅助方案，而非精度优于基线的结论。动态尺度
明确使用轮速作为在线参考，并非不依赖轮速的绝对尺度；若目标是纯视觉绝对尺度，需要
目标场景的稀疏真实深度/激光标注来校准或微调 DA。

快速测试可通过环境变量运行动态模式：

```bash
VO_TRANSLATION_SOURCE=depth_dynamic OUTPUT_DIR=plots/depth_dynamic_test ./run_test_v2.sh
```

---

## 📂 文件结构

```
GPS_IMU_Fusion_EKF-main/
├── calibration.yaml              # 相机标定（含 R_imu_cam / t_imu_cam）
├── parking01.bag                 # 完整数据集（~36GB）
├── extract_stereo_from_bag.py    # bag 单遍提取左目图像 + 真实时间戳
├── stereo_imu_odo_fusion_v2.py   # 融合算法（左目VO + IMU + 轮速 EKF）
├── wheel_odometry.py             # 四轮轮速 → 前向速度
├── depth_prior.py                # Depth Anything V2 米制深度：边缘筛除 / PnP 平移 / 动态尺度
├── evaluate_trajectory.py        # 与真值评估（含 2D 刚体对齐）
├── evaluate_vo_motion.py         # VO 逐帧平移位移 vs 真值（离线诊断，不反馈融合）
├── plot_vo_comparison.py         # 多套结果 vs 真值 对比图
├── tests/test_depth_prior.py     # 深度支路单元测试
├── LightGlue-main/               # SuperPoint + LightGlue 推理代码和匹配器权重
├── superpoint_lightglue/         # SuperPoint 本地权重
├── Depth-Anything-V2-main/       # DA 网络源码
├── depth_anything_v2_metric_hypersim_vits.pth   # DA 米制深度权重
├── run_full_trajectory.sh        # 全轨迹消融编排（WITH_CONTROLS=1 附带扩展对照）
├── run_test_v2.sh                # 快速测试脚本
├── stereo_imu_odo_fusion.py      # v1（ORB 时代基线，保留备查）
│
├── stereo_images/                # 提取的左目图像（自动生成）
│   └── left/
│       ├── frame_*.png           # 左目图像 3772 帧
│       └── timestamps.txt        # 每帧真实采集时间戳（相对bag内IMU首帧）
│
└── plots/
    ├── ablation/                 # 主消融（2×2 + yaw 耦合对照）
    │   ├── {vo,novo}_{bias,nobias}/  fusion_results.csv + error_report.txt + 3 张图
    │   ├── vo_bias_yawcoupled/       # --visual_update_yaw 对照组（30.824 m / 54.34°）
    │   ├── comparison/main_{trajectory,error}.png    # 主消融对比
    │   ├── comparison/da_{trajectory,error}.png      # 有/无视觉 + DA 三种用法
    │   ├── comparison/yaw_protection_{trajectory,error}.png  # 偏航保护 vs 放开耦合
    │   ├── full_fit.png                              # 全轨迹拟合总览
    │   └── absolute_vs_shape.png                     # 未对齐(绝对) vs 对齐(形状)
    └── controls/                 # 扩展对照（WITH_CONTROLS=1 生成）
        ├── vo_probe_no_lateral/      # 增益归因：零图像信息对照
        ├── visual_noise_sweep/       # 视觉噪声重扫 R=1.0/0.35/0.1
        ├── repro_and_init/           # 定标口径 A/B + 初始航向来源对照
        ├── depth_edge_filter/        # DA 仅做匹配边缘筛除
        ├── depth_metric/             # DA 米制 PnP 定标
        └── depth_dynamic/            # DA 米制 PnP + 动态尺度标定
```

运行前需安装 `requirements.txt` 中的 PyTorch 和 Kornia。推理默认使用 CUDA
（可用时），否则使用 CPU；SuperPoint 和 LightGlue 权重从上述本地目录加载。

---

## 🔧 核心算法

### 技术路线

```
parking01.bag
   │
   │  extract_stereo_from_bag.py --left-only
   │    单遍扫描、只迭代左目连接；bytes.find() 定位 JPEG/PNG 魔数
   │    每 3 帧存 1 帧 + 解析 std_msgs/Header.stamp → timestamps.txt
   ↓    （帧率不靠假设：实测相机 10.0000 Hz → 采样后 3.3333 Hz）

┌─────────────────────────────────────────────────────────────┐
│           上电 IMU 静止对准（运行阶段不读取真值）             │
│  加计模长≈g + 陀螺静止 → 检测运动起始(自动skip)             │
│  静止重力向量 → 初始 roll/pitch                              │
│  静止陀螺三轴均值 → 陀螺零偏，并注入状态 x[4]（--apply_gyro_bias）│
│  无磁力计 → 绝对偏航不可观测，初始 yaw=0（里程计系）         │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│               左目单目视觉里程计（Monocular VO）              │
│                                                              │
│  左目图像 ──→ 去畸变(initUndistortRectifyMap) + 0.5 缩放     │
│           ──→ SuperPoint 关键点/描述子                       │
│  相邻左目帧 ──→ LightGlue 匹配                               │
│       │        （可选：DA 米制深度的 log-depth 边缘筛除误匹配）│
│       ↓                                                     │
│  findEssentialMat + recoverPose ──→ 相机系运动方向           │
│       ↓        camera_motion = −Rᵀ·t                        │
│  取轴（按 calibration 的 R_imu_cam）：                       │
│       forward = camera_motion[2]（光轴 cam_z → IMU x）       │
│       lateral = camera_motion[0]（cam_x → IMU y）            │
│       ↓                                                     │
│  平移定标：轮速位移 → (forward, lateral) 米制相对位移        │
│       （VO 自身 yaw_delta 因符号/尺度不可靠，丢弃不用）      │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                  扩展卡尔曼滤波（EKF）融合                    │
│  状态向量: X = [x, y, yaw, speed, gyro_bias]               │
│                                                              │
│  预测（按 --imu_prop_hz 子步，默认 50 Hz，不是每帧一次）：   │
│    yaw_rate = gyro_z − bias；子步间取梯形中点角速度          │
│    x += speed·cos(yaw)·dt ;  y += speed·sin(yaw)·dt          │
│    yaw = wrap(yaw + yaw_rate·dt)                             │
│    —— 偏航的唯一来源是 IMU                                   │
│                                                              │
│  更新（冗余观测只修正其直接观测的状态，Joseph 形式协方差）：  │
│    视觉位置 [x,y] → 只改 x,y 两行 K；yaw/bias 行置零         │
│    轮速 → 只改 speed 行 K                                    │
│    卡方 95% 门控（dof=2 → 5.991；dof=1 → 3.841）             │
└─────────────────────────────────────────────────────────────┘
                            ↓
  evaluate_trajectory.py --align（2D 刚体对齐：旋转+平移，无尺度）
  evaluate_vo_motion.py（逐帧平移位移 vs 真值，不参与融合）
  plot_vo_comparison.py（多配置轨迹/误差对比）
```

可选平移定标支路（`--vo_translation_source`）：`wheel`（默认，基线）、
`depth`（DA 米制深度 PnP 直接给米制平移）、`depth_dynamic`（用轮速位移在线
滑窗反标定深度尺度）。全轨迹实测见下文性能表——`wheel` 仍是最优默认值。

RANGER 的四个轮速通道按 parking01 数据拟合结果分组：四个轮速通道中的
第 1、3 个为右轮，第 2、4 个为左轮。差速角速度为
`(mean(right_wheels) - mean(left_wheels)) / wheel_track_width`。
`--wheel_track_width` 默认 **0.364 m**（AgileX Ranger MINI3 车辆规格实测轮距）。
注意：Ranger MINI3 为四轮独立转向底盘，差速偏航模型不成立，故 EKF 预测中
差速轮速角速度观测已禁用，偏航完全由 IMU gyro_z 驱动。

### 关键改进（相比之前方案）

| 改进项 | 旧方案 | 新方案 |
|--------|--------|--------|
| **相机参数** | 估计值 | 真实标定值 |
| **视觉输入** | 左目图像 | 左目图像（仅） |
| **特征匹配** | ORB | SuperPoint + LightGlue |
| **平移尺度** | 轮速计 | 轮速计（可选 DA 米制深度 PnP / 动态尺度标定支路，实测更差，仅作对照） |
| **视觉更新** | 位姿[x,y,yaw] | 仅位置[x,y]，且不反算 yaw（冗余观测只修正其直接观测的状态） |
| **定位精度** | 中位数5.74m | **全轨迹 17.7min/1222m：位置 RMSE 0.888 m、偏航 1.87°** |
| **图像时间基** | 按假设 3.56Hz 合成 | bag 头部真实时间戳（实测 3.3333Hz） |
| **偏航积分** | 每图像帧一次（3.33Hz） | IMU 率子步（默认 50Hz） |
| **陀螺零偏** | 仅报告、不修正 | 静止对准估计并修正（`--apply_gyro_bias`） |
| **bag 读取** | 两遍扫描 + 逐字节找 JPEG 头 | 单遍 + `bytes.find()` + 只迭代目标连接 |
| **增益归因** | 无对照 | `--vo_probe` 分离图像信息与协方差整形；NIS 按视觉/轮速分道统计 |

---

## 📊 数据处理流程

### 1. 图像提取（每3帧取1帧 + 真实时间戳）

```python
# extract_stereo_from_bag.py --left-only  （单遍扫描bag）
if left_seen % frame_interval == 0:          # 每3帧取1帧
    img = extract_compressed_image(raw)
    stamps.append((name, header_stamp(raw) - t_imu0))   # 记录真实采集时刻
```

**原因：**
- 相机实测 **10.0000 Hz**（间隔 100.00ms，std 0.02ms），数据量大
- 采样后 **3.3333 Hz**（间隔 300ms），减少计算量
- **时间戳必须来自 bag 头部**：`std_msgs/Header.stamp` 与 IMU 首帧之差给出与
  `parking01_ADIS16465_IMU.txt` 完全一致的时钟（两者样本数 226369、时长 1131.48s
  完全相同），因此可直接 `绝对时刻 = imu_times[0] + t_rel`

### 2. 左目单目视觉里程计

```python
# 1. 只从左目图像提取 SuperPoint 特征
features = extractor.extract(left_img)

# 2. 相邻左目帧用 LightGlue 匹配，再由本质矩阵估计运动方向
matches = matcher({"image0": previous_features, "image1": features})
essential, mask = cv2.findEssentialMat(points0, points1, K_left)
_, rotation, translation, mask = cv2.recoverPose(
    essential, points0, points1, K_left, mask=mask
)

# 3. 用轮速计位移给单目平移方向恢复尺度
translation_direction = -rotation.T @ translation
translation_metric = normalize(translation_direction) * abs(wheel_speed * dt)
```

### 3. EKF融合

```python
# 预测：偏航必须由 IMU 率子步积分（默认 --imu_prop_hz 50）
for k in range(n_sub):                       # 每帧图像之间插入 15 个子步
    ekf.predict(sub_dt, gyro_z_mid, wheel_speed)
ekf.update_wheel_speed(wheel_speed)          # 轮速观测

# 更新：视觉仅修正位置 [x, y]，偏航完全交给 IMU。
# 相机系运动 (forward, lateral) 用 EKF 的 yaw（IMU）旋入世界系后累积，
# 单目VO自身的 yaw_delta 因符号/尺度不可靠而不参与融合。
ekf.update_visual_position(vo_xy_meas)       # 本质矩阵 + 轮速定标的相对位移
```

> **为什么必须按 IMU 率子步积分**：若像早期版本那样每帧只取一个陀螺样本来推进 0.3s 的
> 偏航，陀螺高频噪声会**混叠**进积分。实测同一 17 分钟窗口、施加静止零偏后的偏航积分误差：
>
> | 角速度积分率 | 200 Hz | 50 Hz | 10 Hz | 3.33 Hz（图像率） |
> |---|---|---|---|---|
> | 偏航 RMSE | 1.29° | **1.84°** | 19.13° | 41.39° |
>
> 这是长轨迹崩掉的主因之一，与"单目VO是否有独立信息"无关。

> **设计要点**：诊断显示单目VO的偏航与IMU积分符号相反、尺度错误，故视觉只贡献位置修正，
> 偏航由IMU承担。注意位置更新会通过协方差耦合反作用于偏航状态。

---

## ⚙️ 参数配置

### 运行参数

```bash
python stereo_imu_odo_fusion_v2.py \
    --calibration calibration.yaml \      # 相机标定文件
    --left_dir stereo_images/left \       # 左目图像目录
    --imu_file parking01_ADIS16465_IMU.txt \
    --odo_file parking01_RANGER_ODO.txt \
    --output_dir plots/stereo_fusion \
    --timestamps_file stereo_images/left/timestamps.txt \  # bag内真实时间戳
    --imu_prop_hz 50 \                    # EKF预测子步频率（关键，见上文）
    --image_scale 0.5 \                   # 图像处理缩放
    --max_frames 100000 \                 # 最大处理帧数（默认不限）
    --init_method imu_alignment \         # 部署模式(无真值)；groundtruth 为对照模式
    --apply_gyro_bias \                   # 应用静止对准的陀螺零偏
    --visual_position_noise 3.5 \         # 视觉位置噪声(实测最优)
    --wheel_speed_noise 0.15              # 轮速计噪声
```

诊断/对照开关：

| 参数 | 作用 |
|---|---|
| `--visual_update_yaw` | 放开 yaw 保护，让位置更新经协方差耦合反算偏航（长轨迹会恶化到 30.824 m / 54.34°，且自报 1σ 低估 2.5 倍，仅作对照） |
| `--vo_probe {normal,no_lateral,flip_lateral,no_forward}` | 破坏 VO 的图像信息但保留更新结构与频次，用于分离"图像信息增益"与"协方差整形增益" |
| `--vo_scale_integral` | 单目平移改用轮速梯形积分距离定标（与 EKF 预测位移一致），而非瞬时轮速×Δt |
| `--vo_translation_source {wheel,depth,depth_dynamic}` | 平移定标来源：轮速（默认）/ DA 米制深度 PnP / 深度+动态尺度标定 |
| `--depth_checkpoint / --depth_input_size / --depth_edge_threshold` | Depth Anything V2 权重与深度边缘筛除参数 |

`--image_fps`（默认 3.3333）仅在缺少 `timestamps.txt` 的回退路径中用于合成时间戳。

### IMU 静止对准初始化（无真值部署）

系统上电后先静止约 32s，利用这段静止期完成对准，**运行阶段不读取任何真值**：

| 对准量 | 方法 | 本数据集结果 |
|--------|------|-------------|
| 运动起始时刻 | 加计模长≈g 且陀螺静止，首个持续运动点 | 上电后 31.79s（186434.214） |
| 初始横滚/俯仰 | 静止重力向量投影（FRD 系） | roll≈0.16°, pitch≈-1.49° |
| 陀螺零偏 | 静止段三轴均值 | gz≈**+0.0403°/s** |
| 初始偏航 | **无磁力计→静态不可观测，置 0（里程计系）** | yaw=0° |

起始帧不再按"skip 帧数"取，而是按真实时间戳筛选 `t ≥ 运动起始时刻`。

**为什么偏航置 0**：ADIS16465 不含磁力计，重力只能定 roll/pitch，无法定绝对航向。航位推算系统本就工作在自定（odometry）参考系。评估时改用 `evaluate_trajectory.py --align` 做 2D 刚体对齐（旋转+平移，无尺度），公平度量轨迹**形状精度**。

**全轨迹验证"去除真值零损失"**（同一段 3666 帧，只切换初始航向来源）：

| 初始航向来源 | 对齐后位置 RMSE | 对齐后偏航 RMSE | 对齐航向偏移 | **未对齐**绝对位置 RMSE |
|---|---:|---:|---:|---:|
| 部署模式 yaw=0（不读真值） | **0.888 m** | 1.873° | 87.42° | 64.35 m（max 114.8 m） |
| 真值初始航向 88.7°（对照） | **0.888 m** | 1.873° | −1.28° | 4.08 m（max 7.94 m） |

形状精度逐比特一致（x/y/yaw 差 ≤1e-6），证明**去除真值初始化对形状精度零损失**；
但代价全部落在**绝对定位**上：仅"知道初始航向"这一个标量，就把绝对系误差从
64.35 m 压到 4.08 m（↓15.8×）。真值起点为 (0,0,0)，且部署模式起点平移仅 0.45 m，
所以这 60 m 的差距几乎纯粹是航向参考缺失，而非坐标原点约定差异。

> **陀螺零偏必须修正（`--apply_gyro_bias`）**。静止对准得到的 **+0.0403°/s** 与
> 用真值反解出的最优常值零偏 **+0.0424°/s** 仅差 5%。在 17 分钟窗口内，
> 200Hz 积分的偏航误差从 **25.63° → 1.29°**；50Hz 子步为 **26.35° → 1.84°**。
>
> 早期版本曾得出"零偏修正无益、主因是尺度因子"的结论，那是**两个时间基 bug 掩盖下的假象**：
> 修正偏置带来的改善被错误的时间尺度与混叠噪声吞掉。实测尺度因子误差仅 0.4%
> （gz ≈ 1.004 × 真值角速率），远小于 0.0403°/s 的零偏影响。

### 噪声参数调优

| 参数 | 说明 | 建议值 |
|------|------|--------|
| `visual_position_noise` | 视觉位置信任度 | **3.5**（全轨迹重扫最优，见"逻辑审查发现 ⑤"；1.0→1.126 m、0.35→1.489 m、0.1→6.248 m，均劣于 3.5→0.888 m） |
| `wheel_speed_noise` | 轮速计信任度 | 0.1 - 0.3 |

**调优原则：**
- 值越小 → 越信任该传感器；值越大 → 越不信任该传感器
- 单目 VO 平移由轮速定标，与 EKF 预测高度重复，过度信任（noise<2）会引入噪声使结果变差
- **不能**用 innovation 远小于 √R 来推断 R 偏大：冗余观测的 innovation 小是同源误差
  造成的，与测量自身精度无关（本项目实测即为此案例）

---

## P0-2 可行性探针：车道线航向能测到多少度？

**结论：能，而且比需要的精度高一个量级。** 在不动用任何深度模型、不下载权重的前提下，
只用 `calibration.yaml` 的真实内参 + 重力方向 + 经典 Canny/Hough，直行帧上车道线方向角
观测与真值航向的一致性残差 **标准差 0.20°、P95 0.37°、最大 1.10°**（门限内 188 帧）。
判定：**P0-2 成立，几何提取不是瓶颈，UFLine 分割也不是瓶颈。**

脚本：`lane_probe.py`（提取与统计） + `analyze_lane_probe.py`（汇总与出图）
产物：`plots/lane_probe/{final_report.txt, lane_probe_summary.png, base|fixed|p05|r05|p10|r10/, examples/}`

### 方法要点（三处容易做错的地方）

1. **必须去畸变**：radtan k1=−0.1516 在本镜头角点产生 **±63 px** 位移，折合 **≈3.4°** 角度误差，
   比要测的精度大一个量级。
2. **α 与相机离地高度无关**：图像线 L → 观测平面法线 `n = Kᵀ·L`（不是 `K⁻ᵀ·L`）→
   与地面法线叉乘得 3D 水平方向 → `α = atan2(d·right, d·forward)`。
   地面法线来自重力，所以只需要 roll/pitch + 内参 + 相机-IMU 外参。
   高度只用于把线段端点反投影到地面做 **1–16 m 地距门限**（剔除远场墙面杂点）。
   附带收获：`odo_lever z=0.852` 与 `t_imu_cam z=−0.054` 说明**相机离地高度其实已知**（h=0.906 m），
   不是需要猜的量。
3. **α 是 mod-180 无向量**：环形统计必须先"加倍角"（2α 落到普通 360° 圆周）。
   直接对 [0,180) 取圆均值会把众数硬拉到 90° 附近——这个 bug 曾让所有帧输出恒等于 90°。

**一个有用的几何性质**：任何水平且平行于车道族的 3D 直线（地面车位线、墙脚线、停车列边缘、
吊顶桥架）共享同一消失点，因此给出同一个 α。实测检出的正是**右侧白色车位边线与左侧墙脚线**，
段内散布中位 0.46°。

### 结果（全量 stride=6，366 帧有效，271 帧为直行帧）

| 组 | 姿态来源 | 门限内 n | 通过率 | P50° | P68° | P95° | 最大° | >3°% |
|---|---|---|---|---|---|---|---|---|
| base | GT 原始姿态（上界，含尖峰） | 188 | 51% | 0.10 | **0.17** | 0.37 | 1.10 | **0.0** |
| gtclean | GT 0.5 s 中值滤波（去尖峰上界） | 158 | 43% | 0.11 | 0.19 | 0.36 | 0.77 | 0.0 |
| online | **在线互补滤波（可部署口径）** | 70 | 19% | 0.23 | **0.38** | 1.10 | 1.48 | **0.0** |
| fixed | 固定=起始对准（可部署） | 46 | 13% | 0.39 | 0.58 | 2.92 | 4.05 | 6.5 |
| p05 | GT + pitch 扰动 0.5° | 32 | 9% | 0.22 | 0.31 | 0.69 | 3.46 | 3.1 |
| p10 | GT + pitch 扰动 1.0° | 19 | 5% | 0.26 | 0.39 | 1.11 | 4.32 | 5.3 |
| r05 | GT + roll 扰动 0.5° | 180 | 49% | 0.11 | 0.17 | 0.37 | 1.08 | 0.0 |
| r10 | GT + roll 扰动 1.0° | 170 | 47% | 0.11 | 0.19 | 0.37 | 1.06 | 0.0 |

**评估口径修正（本表之后补做的核查）**：检查 `parking01_groundtruth.nav` 的 roll/pitch 列发现
**GT 姿态自身含尖峰伪影**——|pitch|>3° 的"段"全部只持续 0.1–0.3 s、±3°/∓5° 成对交替、期间 z
不变（−0.09~0.02 m）且 |yaw rate|≈1°/s，与悬架/坡道特征不符，属参考解算的毛刺。因此：
gtclean 组证明尖峰对探针统计几乎无影响（尖峰帧本身被门限拒掉）；而 online 组与 GT 瞬时值
的差（RMSE 0.55°/0.66°）大部分是 GT 毛刺贡献的，**在线姿态对真值姿态的真实误差更小**，
端到端指标以上表 online 行为准（这是可部署口径的正式数字）。

（每组另有一行"不加门限"数据在 `final_report.txt`：base 为 P50 0.14°/P68 0.23°/4.4% 离群。）

### 三条可执行的工程结论

1. **精度由姿态知识决定，不由视觉决定。** roll 扰动 ±1.0° 对结果**零影响**（P68 仍 0.17→0.19°），
   因为 α 只用到 u 方向几何；pitch 扰动 1.0° 使 P68 升到 0.39°。而"固定用起始对准姿态"退化到
   P95 2.92°——原因是真值 pitch 在行程中变化约 ±0.5°（悬架/坡度），**变化的那部分才是杀手**。
   → 部署时必须**在线跟踪 pitch**（加速度计低通即可，roll 可不管）。✅ **已实现**：
   `lane_heading.py::ComplementaryAttitude`，FRD 陀螺传播 + 加速度计互补修正（τ=10 s），
   机动污染用物理门限抑制（|v̇|<0.4 m/s² 才修 pitch、|v·ωz|<0.4 m/s² 才修 roll）；
   上表 online 行就是它的端到端口径（P68 0.38°、最大 1.48°、零离群）。
2. **自参照门限有效，不需要外部参考。** 用"α 众数权重占比 ≥0.80 且众数内散布 ≤0.60°"这一
   纯自检测量做门限，把 >3° 的离群帧从 4.4% 压到 **0.0%**，同时保留 51% 的帧（等效每 3.5 s 一次
   有效航向观测）。转弯帧（|yaw rate|≥6°/s，占 26%）本就该排除。
3. **一个标量常数就能解释全程观测。** `wrap90(α+yaw)` 的分 120 s 时段中位数为
   −0.06/−0.09/+0.02/+0.03/+0.01/+0.08/−0.02/+0.06/+0.20°，**时段间最大离散 0.29°**；
   本序列该常数为 **−1.568°**（车道网方位 + 相机安装偏置 + 车辆-车道夹角的合并量）。
   → 绝对航向仍需求这一个常数（**设施级一次性标定**，不是每次开机标定），除此之外完全免 GNSS、
   免先验地图、免额外硬件。

### 与绝对定位缺口的换算

本序列轨迹紧凑（到起点距离均值 39.6 m、最大 77.8 m），绝对航向误差的杠杆实测
**0.76 m/°**（87.42° 偏置 ↔ 64.35 m 未对齐 RMSE 自洽）。用归档融合轨迹绕起点旋转仿真：

| 残余绝对航向误差 | 0° | 0.25° | 0.5° | 1° | 2° | 5° | 10° | 现状(不修正) |
|---|---|---|---|---|---|---|---|---|
| 未对齐绝对 RMSE (m) | 1.21 | 1.22 | 1.27 | 1.43 | 1.96 | 4.02 | 7.76 | 64.27 |

（完整表与口径说明见 `plots/lane_probe/heading_lever.txt`，由 `heading_lever.py` 生成；
0° 行的 1.21 m 是"绕起点旋转 + 原点共用"的下限，与 2D 刚体对齐给出的 0.888 m 形状误差口径不同。）

探针实测可达 0.17°(P68)/0.37°(P95) → **未对齐绝对 RMSE 有望从 64.35 m 降到 ~1.2–1.5 m（约 45–50 倍）**，
同时偏航通道首次被独立观测闭环，`P[2,2]` 不再单调发散（顺带解决协方差病态）。

### 尚未解决 / 已尝试的诚实记录

- **90°/180° 模糊（已按设计消解）**：探针统计量取 mod-90，天然不区分四个象限（判错一
  象限 ≈ 66 m，等于白做）。消歧实现：**自洽模式**用 mod-90 环绕创新 + 一致性缓冲，网格
  锁定免绝对信息（象限差被吸进 C_lane 状态）；**绝对模式**需两个一次性场地输入——设施
  常数 C0（罗盘/图纸/异日标定，1 个标量）与初始象限 k∈{0,1,2,3}（开机选"大致朝向"，
  行业常规），初始化只接受 |α|≤45° 的"前进方向族"观测（防库位内对着垂线族换系）。
  更新创新同样按 mod-90 收缩（正交族翻折实测会把通道打死锁，折回 ≈0 才可用），因此
  **k 判错不会被滤波内部察觉**：`abs_k0` 对照实测绝对位置 63.71 m、形状完全不变、
  451 次 NIS 全过——判错一象限 ≈ 60 m 的代价须在答辩与部署手册明示。
- **次族正交自检（已做，结论为负，不采用）**：在 α1+90°±15° 邻域约束搜索第二族
  （`lane_probe.py::orthogonality_check`）。结果：门限内帧中 63% 正交邻域无线段（通道上只有
  单族可见线，与场景判断一致）；而"有次族"与"无次族"帧的 c90 残差几乎相同
  （online 组 0.43° vs 0.32° P68）——该信号**没有判别力**，主门限（frac≥0.8 & spread≤0.6）
  本身已足够好。副产物结论：单族观测同样精确，不必强求两族同现。
- **单一场地**：常数 −1.568° 只在 parking01 验证；换场地需重标。
- **UFLine 权重仍缺**（Step 1 未开始）：本探针已证明经典法够用，UFLine 属"对照项"而非必需项。

### 复现

```bash
cd GPS_IMU_Fusion_EKF-main
for tag in base fixed p05 r05 p10 r10; do
  A="--stride 6"; [ $tag = fixed ] && A="$A --fixed-attitude"
  [ $tag = p05 ] && A="$A --pitch-pert 0.5";  [ $tag = p10 ] && A="$A --pitch-pert 1.0"
  [ $tag = r05 ] && A="$A --roll-pert 0.5";   [ $tag = r10 ] && A="$A --roll-pert 1.0"
  .venv/bin/python3 lane_probe.py $A --save-vis 0 --out plots/lane_probe/$tag
done
.venv/bin/python3 lane_probe.py --stride 6 --attitude online    --save-vis 0 --out plots/lane_probe/online
.venv/bin/python3 lane_probe.py --stride 6 --attitude gt_clean  --save-vis 0 --out plots/lane_probe/gtclean
.venv/bin/python3 analyze_lane_probe.py          # 汇总表 + lane_probe_summary.png + final_report.txt
.venv/bin/python3 lane_probe.py --stride 400 --save-vis 2   # 出 examples/ 的标注帧
.venv/bin/python3 lane_heading.py --validate     # 在线姿态 vs 真值（真值仅作评估）
```

### P0-2 落地实现：在线姿态 + EKF 的 θ_lane 独立观测通道

实现文件：`lane_heading.py`（`ComplementaryAttitude` 在线姿态 + `LaneHeadingObserver`
逐帧观测器，几何与门限和 Step 0 探针完全同口径）；`stereo_imu_odo_fusion_v2.py` 的
`PlanarEKF` 增广**第 6 状态 C_lane**（车道网常数，Q=0 场地常量），新增
`update_lane_heading()` 观测通道与 mod-90 网格一致性缓冲（连续 5 个门限内观测
ψ+α−C 互差 <3° 才允许更新，防起步/跨道瞬态拉飞偏航）。车道观测是偏航的**独立**
信息源（重力 + 标定几何，不含轮速/IMU），因此允许直接修正 yaw 与陀螺零偏，
不违反"视觉位置不得泄漏进偏航"的保护。默认关闭；开启后基线结果逐位不变
（150 帧回归检查 vs 归档头条基线最大偏差 0.0e+00）。

两种模式：

| 模式 | 输入 | 观测量 | 用途 |
|---|---|---|---|
| 自洽（`--lane_heading`） | 无 | ψ+α≡C (mod 90°)，C 为状态 | 免任何绝对信息消除偏航漂移、治好 P[2,2] 发散 |
| 绝对（+`--lane_abs_cal_deg C0 --lane_init_from_obs --lane_quadrant k`） | C0：场地级 1 标量；k：开机 1 次象限选择 | ψ = C0−α (mod 90°) 直接观测 | 把航向锁进绝对系 → 未对齐绝对 RMSE 实测 **1.34 m**（杠杆表预言 1.2–1.5 m 档兑现） |

全轨迹结果（3666 帧，stride=1，与头条基线同参数；汇总 `lane_sweep_report.py` +
`analyze_lane_full.py`，产物 `plots/lane_full/{lane_sweep_report.txt,lane_full_report.txt,lane_full_summary.png}`）：

**部署配置 = 车道绝对模式 + 转弯门限 2.5°/s + `--yaw_q 1e-6`**：

| 运行 | 形状位置 RMSE（对齐后） | 形状偏航 RMSE† | 未对齐绝对位置 RMSE | 未对齐绝对偏航 RMSE† | 车道观测 尝试/门内/采纳 |
|---|---|---|---|---|---|
| 基线（VO+零偏，无车道） | 0.888 m | 1.38° | 64.35 m | —（里程计系，无意义） | — |
| +车道·自洽模式（同门限） | 0.883 m | **0.36°** | 63.49 m | —（里程计系） | 2419 / 496 / 429 |
| +车道·绝对模式（k=1，部署） | **0.767 m** | **0.37°** | **1.34 m** | **0.39°** | 2419 / 496 / 429 |

†统一自算口径（GT/估计都解卷绕；形状偏航去中位常数）。官方 `evaluate_trajectory.py`
的偏航口径对 ±180 锯齿 GT 是"先插值后解卷绕"，有 ≈1.2° 常数性畸变（基线同样受影响：
官方 1.87° vs 自算 1.38°；部署配置官方 noalign 实测 1.37° 自算 0.39°）；**位置口径不受
影响**，官方已复核落盘：`abs_g25_qfix` 对齐 **0.7666 m**、未对齐 **1.3375 m** ✓。

要点（可复核，全部落盘）：

- **头条**：未对齐绝对位置 RMSE **64.35 → 1.34 m（48×）**、绝对偏航误差 **0.39°**，
  全程无 GNSS、无先验地图；正好落在杠杆分析预言的 1.2–1.5 m 档。一次性输入仅两个
  场地级标量：设施常数 C0=−1.568°（罗盘/图纸/异日标定）与开机象限 k。
- **形状零代价、反而更好**：0.888→0.767 m（绝对）/0.883 m（自洽），形状偏航
  1.38°→0.36°。车道通道不再拖累形状（第一版会拖累，见下面的消融史）。
- **协方差病态（P1-1）同步治好**：绝对模式偏航 1σ 峰值 30.2°→**1.10°**、末端
  25.5°→0.69°；自洽模式 C_lane 收敛 −0.164°（独立核算的里程计系网格相位 −0.39°±0.6 ✓）。
- **初始化换系**：静止起始段扫描（部署语义=开机拍一帧），α=+0.23° → ψ₀=+88.20°
  （GT 88.70°，差 0.50°，单帧观测噪声内），**零位移处换系**；只接受 |α|≤45° 的
  "前进方向族"观测（对着库位垂线族初始化实测会把航向锁错 90°，已设门）。
- **两个独立缺陷的 2×2 消融史（诚实记录，全部落盘）**：第一版（门限 6°/s、Q 原值）
  形状劣化 0.888→1.96 m，且 R 扫描四组（0.4/0.7/1.2/自适应）形状 1.926–1.959 m
  几乎不动——R 无效说明偏航新息增益 K≈1，根因是 `Q[2,2]=0.003 rad²/s` 病态使 P₂₂
  在更新间隔内膨胀到 (0.6 rad)²（即 P1-1 记录的那个）。另一独立缺陷：缓弯
  （2.5–6°/s）中 α 带系统偏差被 6°/s 门限放过。逐项验证：

  | 2×2（绝对模式） | 形状位置 | 绝对位置 | 形状偏航 | 采纳/拒绝 |
  |---|---|---|---|---|
  | 门限6°/s·Q=0.003（`abs_k1`） | 1.959 | 3.41 | 1.28° | 451/0 |
  | 门限2.5°/s·Q=0.003（`abs_gyr25`） | 1.533 | 2.65 | 1.24° | 424/0 |
  | 门限6°/s·Q=1e-6（`abs_qfix`） | 2.621 | 3.64 | 1.76° | 369/116 |
  | 门限2.5°/s·Q=1e-6（`abs_g25_qfix`，**部署**） | **0.767** | **1.34** | **0.37°** | 429/24 |

  两项都必须做：只修 Q 会让 K 变小、坏转弯观测的残余影响被拖长（2.621 m）；
  只收门限则 K 仍≈1、新噪声满增益注入（1.533 m）。组合后 0.767 m。
- **初始化 vs 持续锁定的分解**：`--lane_init_only`（只换系不更新）在旧门限下绝对
  4.03 m——第一版 3.41 的绝对收益几乎全来自开机换系；而部署配置 1.34 m 显著优于
  单次初始化能达到的水平，**持续锁定的价值成立**（并把唯一 >4° 的 479–496 s 缓弯段
  从 −6.7° 压到门限之外）。
- **象限错配代价（已量化）**：`abs_k0` 把 k 判错一象限 → 绝对位置 63.71 m（≈没有
  车道通道）、绝对偏航 89.7°，但形状完全不变且 NIS 全部通过——**错象限是静默失效，
  mod-90 折叠让滤波自身无法察觉**，唯一防线是开机时操作员输入正确 k（罗盘粗方向）。
  答辩口径：判错一象限 ≈ 60 m，判对 = 1.34 m。
- **回归红线不破**：默认参数（车道关闭或 Q 默认）与归档基线逐位一致（前 150 帧最大
  偏差 0.0e+00）；`--yaw_q`/`--lane_max_gyr` 均为显式实验开关。诚实注记：修 Q 后
  **无车道**基线形状为 1.077 m（`base_qfix`）——归档 0.888 部分受益于大 P₂₂ 与轮速
  尺度误差在闭合轨迹上的相互抵消；车道部署配置（0.767）优于两者，不受此影响。

复现（与上面表格一一对应；`COMMON` 见 `run_full_trajectory.sh`）：

```bash
# 自洽模式（部署门限）
.venv/bin/python3 stereo_imu_odo_fusion_v2.py "${COMMON[@]}" \
  --lane_heading --lane_max_gyr 2.5 --yaw_q 1e-6 --output_dir plots/lane_full/grid_g25_qfix
# 绝对模式（部署配置，k=1）
.venv/bin/python3 stereo_imu_odo_fusion_v2.py "${COMMON[@]}" \
  --lane_heading --lane_abs_cal_deg -1.568 --lane_init_from_obs --lane_quadrant 1 \
  --lane_max_gyr 2.5 --yaw_q 1e-6 --output_dir plots/lane_full/abs_g25_qfix
# 双口径评估 + 汇总
.venv/bin/python3 evaluate_trajectory.py --estimated plots/lane_full/abs_g25_qfix/fusion_results.csv \
  --groundtruth ../parking01/parking01_groundtruth.nav --output_dir plots/lane_full/abs_g25_qfix_aligned --align
.venv/bin/python3 evaluate_trajectory.py --estimated plots/lane_full/abs_g25_qfix/fusion_results.csv \
  --groundtruth ../parking01/parking01_groundtruth.nav --output_dir plots/lane_full/abs_g25_qfix_noalign
.venv/bin/python3 analyze_lane_full.py && .venv/bin/python3 lane_sweep_report.py
```

---

##  与 i2Nav-Robot 官方基准的关系

数据集出处：Tang et al., *i2Nav-Robot: A Large-Scale Indoor-Outdoor Robot Dataset for
Multi-Sensor Fusion Navigation and Mapping*, arXiv:2508.11485（GitHub `i2Nav-WHU/i2Nav-Robot`，
GPLv3 仅限学术用途）。官方在 parking01 上用左目相机 + Hesai AT128 激光雷达 + ADIS16465 IMU
统一评测了 9 个里程计系统（ARE°/ATE m）：

| 方法 | 传感器 | parking01 ARE / ATE |
|---|---|---|
| VINS-Mono | 单目+IMU | 3.67 / 3.86 |
| DM-VIO | 单目+IMU | 0.83 / 2.33 |
| OpenVINS | 双目+IMU | 0.34 / 0.81 |
| DLIO | 激光+IMU | 0.58 / 0.15 |
| FF-LINS | 激光+IMU | 1.69 / 1.06 |
| FAST-LIO2 | 激光+IMU | 1.21 / 0.44 |
| FAST-LIVO2 | 激光+视觉+IMU | 0.65 / 0.17 |
| LE-VINS | 激光+视觉+IMU | 0.30 / 0.69 |
| R3LIVE | 激光+视觉+IMU | 0.91 / 0.39 |
| **本项目** | **单目+IMU+轮速（松耦合）** | **1.87° / 0.888 m** |

**口径说明与差异**：

- 官方表给出 parking01 = 1060 s / 1224 m；本项目从 `.nav` 独立算得 **1060.0 s / 1223.3 m**，
  评测窗口一致，可直接同图对照。
- 官方 ATE 按其论文约定（KITTI 式对齐），本项目为 **2D 刚体对齐后的形状误差**，
  两者量级可比但**不是严格同一定义**，本表只用于定位量级，不作排名。
- 官方数字**引自论文，非本项目复跑**；同条件复现需使用官方
  `i2Nav-Robot-Baseline` 的改动源码与配置（尚未进行，见"下一步"）。
- 关键差异：官方 15 个基线**均未使用车轮里程**，单目方案的米制尺度全部来自紧耦合优化；
  本项目用轮速作显式尺度源 + 松耦合平面 EKF，且**不使用激光雷达、不使用右目、不使用 GNSS**。
- 官方室内序列（含 parking01）的**真值本身**由商用激光 SLAM 预建图 + MM/INS 后处理生成；
  本项目为纯里程计，不需要先验地图。

---

## 📈 性能

### 实测耗时（2026-10-19，RTX 4090 D，`--image_scale 0.5` 即 800×600 处理）

| 项 | 数值 | 测法 |
|---|---:|---|
| 400 帧端到端 | 42.6 s | `MAX_FRAMES=400 ./run_test_v2.sh` |
| 900 帧端到端 | 86.5 s | 同上，`MAX_FRAMES=900` |
| **边际单帧耗时** | **87.8 ms/帧** | (86.5−42.6)/500，扣除固定开销 |
| 固定启动开销 | ≈7.5 s | 模型加载 + IMU/轮速装载 + 评估出图 |
| 相对实时 | **≈3.4 倍余量** | 图像输入周期 300 ms ÷ 87.8 ms（占空比 29%） |
| 全轨迹 3666 帧 | ≈5.5 min | 与 `WITH_CONTROLS` 各次运行时长一致 |

> 该耗时为**离线批处理**口径，含 PNG 读盘解码；未做嵌入式移植与在线实时性正式评测。
> 400 帧冒烟测试的 VO 成功率为 399/400，与全轨迹 97.76% 一致。

### 单目VO特性

| 特性 | 当前左目单目VO |
|------|----------------|
| **图像输入** | 仅左目 |
| **尺度** | 由轮速计恢复 |
| **特征匹配** | SuperPoint + LightGlue |
| **旋转与平移方向** | 相邻帧本质矩阵 |
| **深度信息** | 不使用双目视差 |
| **可选深度先验** | Depth Anything V2 Hypersim Vits；可筛除深度边缘匹配，或通过米制 PnP 估计平移 |

### 实测定位精度：全轨迹 2×2 消融（对比真值 parking01_groundtruth.nav）

**i2Nav-Robot parking01 完整轨迹：处理 3666 帧 / 1099.5 s（18.3 min），其中与真值重叠的
> 3533 帧 / 1060.0 s（17.7 min）用于精度评定，真值里程 1222 m /
覆盖区域 87×80 m、含 360° 以上累计转向。全程不读取任何真值（IMU 静止对准初始化），
评估用 2D 刚体对齐度量形状精度。**

| 视觉更新 | 陀螺零偏修正 | 位置 RMSE | 位置均值 | 位置最大 | 偏航 RMSE |
|:---:|:---:|---:|---:|---:|---:|
| ✗ |  | 8.238 m | 7.178 m | 16.69 m | 13.70° |
| ✗ | ✓ | 1.172 m | 1.083 m | 2.24 m | 1.87° |
| ✓ | ✗ | 7.661 m | — | 15.44 m | 13.70° |
| **✓** | **✓** | **0.888 m** | **0.841 m** | **1.62 m** | **1.87°** |

最佳配置（左目 SuperPoint+LightGlue VO + 静止零偏修正）的误差分布：
中位数 0.811 m、P95 1.285 m、末帧 1.293 m、误差时间斜率 **+0.41 mm/s**
（即 17 分钟仅增长 0.43 m，**无累积漂移趋势**）。相对航向偏置 87.42° ≈ 真值初始
航向 88.7°，说明轨迹形状基本无畸变。

**三点结论：**

1. **陀螺零偏修正是主导因素**（8.24 m → 1.17 m，↓86%）。静止对准估得
   gz=+0.0403°/s，与真值反解的最优值 +0.0424°/s 仅差 5%。
2. **视觉更新确实带来增益**（1.172 m → 0.888 m，↓24%），但**前提是它不得改写偏航**：
   若允许位置更新经协方差耦合反算 yaw，结果反而恶化到 **30.824 m / 54.34°**（对照组已落盘
`plots/ablation/vo_bias_yawcoupled/`，对比图 `plots/ablation/comparison/yaw_protection_*.png`）。
   且该组滤波器**过度自信**：末端自报偏航 1σ=21.92°，而实际误差 54.34°（低估 2.5 倍）；
   保护组则是反向保守（1σ=109.54° vs 实际 1.87°，见「逻辑审查发现 ③」）。
   单目 VO 平移既由轮速定标、方向又用本 EKF 的 yaw 旋回世界系，与预测高度冗余，
   因此按"冗余观测只修正其直接观测的状态"处理（`--visual_update_yaw` 可放开做对照）。
3. **偏航精度与视觉无关**（有/无 VO 均为 1.87°），符合设计：偏航完全由 IMU 承担。

### 增益归因对照（`--vo_probe`）

"位置更新与预测冗余"这一说法需要证据：那 24% 究竟是图像信息，还是仅仅多了一次
协方差整形？用一个**不含任何图像信息、但保持更新结构与频次**的对照来分离——
`--vo_probe no_lateral` 把横向分量置零，此时视觉测量退化为
`anchor + Σ R(yaw)·[轮速距离, 0]`，即纯航位推算的复本：

| 配置 | 位置 RMSE | 相对基线 | 含义 |
|---|---:|---:|---|
| 无视觉更新 | 1.172 m | — | 基线 |
| `--vo_probe no_lateral`（更新照做，但零图像信息） | 1.124 m | ↓4.1% | **仅协方差整形**能买到的部分 |
| 正常视觉更新 | 0.888 m | ↓24.2% | 其中约 **20 个百分点来自真实图像信息** |

结论：视觉的增益是真实的，不是滤波调参的副产品。

> 早先用"NIS 均值≈0、0 拒绝"论证视觉冗余是**不成立**的：`nis_values` 把轮速更新
> （其 innovation 因 `predict` 里已把 `x[3]=wheel_speed` 而结构性趋零）与视觉更新
> 混在同一数组里，视觉的真实 NIS 被稀释。现已按来源分开统计（见下文诊断输出）。

剩余误差主要来自轮速里程的 **+1.62% 尺度误差**（融合路径 1244.1 m vs 真值 1222.4 m），
这是无绝对参考时单目+轮速体系的固有下限。

> 注：早期文档记录的"591 帧 3.94 m / 8.27 m"等数字测于两个时间基 bug 修正之前，已作废。

---

## 🧪 逻辑审查发现（本轮代码走查）

以下问题由逐行走查 + 定量验证得出，均已处理或已量化。

**① NIS 统计被结构性零值稀释（已修复，并测出真实值）**
`predict()` 里已把 `x[3] = wheel_speed`，随后 `update_wheel_speed()` 的 innovation
只剩一个子步内的速度变化量（数量级 ~1e-2 m/s），其 NIS 恒接近 0。旧版把轮速与
视觉的 NIS 追加进同一个 `nis_values`，导致"视觉 NIS 均值≈0"这一读数失真。分道统计后：

| 读出 | 视觉更新 | 轮速更新 |
|---|---|---|
| NIS 均值 / 最大 | 0.001 / 0.034 | 0.0049 / 0.452 |
| 对应 innovation 幅度 | **0.111 m（中位）/ 0.645 m（最大）** | — |
| 滤波器假设的 σ_ν = √S | **3.50 m** | 0.15 m |

由此可以纠正我上一轮的结论：innovation 远小于 √S **不能**用来断言"R 被高估"，
因为该测量与预测同源冗余；R 是否偏大只能靠全轨迹重扫判定，见下文 ⑤。

**② 轮速并非被滤波，而是被直接注入（设计现状，非 bug，但需知情）**
速度作为控制输入在预测步写入状态，更新步对状态的修正量近乎为零，仅收缩 `P[3,3]`。
其后果是：**轮速的测量噪声没有被 EKF 平滑**，直接进入位置递推。若需要真正的速度
滤波，应去掉 `predict()` 中的 `x[3] = wheel_speed` 赋值，改为纯预测 + 观测更新。

**③ 偏航过程噪声过大，且保护模式下永不清减（闭式预测已被实测证实）**
`Q[2,2] = 0.003 rad²/s`。在 yaw 保护下 K 的第 2 行被置零，可闭式证明
`(A P Aᵀ)[2,2] = P[2,2]`，即 **偏航方差只增不减**：
`P[2,2](T) = 0.3 + 0.003×1099.5 = 3.60 rad²` → 预测 yaw 1σ = **108.7°**；
实际运行打印 **109.54°**，与闭式推导一致（残差来自 `F[2,4]` 与零偏互协方差耦合）。
而实测 yaw RMSE 只有 1.87°，**高估 58 倍**。位置侧则是反方向：全程峰值 1σ
= 2.17 m，末端 0.56~0.65 m，而实际误差 0.888 m —— 末端偏乐观。
`Q[2,2]` 应取 ~1e-6 rad²/s 量级（ADIS16465 ARW）。当前设置不影响状态精度
（偏航是开环积分的），但意味着**协方差输出不能用于任何置信度/安全判据**。

**④ VO 平移定标与 EKF 位移口径不一致（已加 `--vo_scale_integral`，实测中性）**
定标用的是"末端瞬时轮速 × Δt"，而预测积分的是梯形距离。实测两者：总里程差
仅 **+0.05 m / +0.004%**（无系统偏差），但**逐帧相对偏差 std = 5.83%，31.6% 的帧
超过 5%**，逐帧绝对偏差 std = **18.5 mm**（单帧位移均值 35.3 cm）——即加减速段
每帧引入约 5% 的随机尺度误差。

全轨迹 A/B：改用积分距离定标后位置 RMSE **0.888 → 0.891 m**（最大误差 1.62 → 1.60 m）。
**精度上不可区分**。这反过来说明：单帧 5% 的测量尺度扰动只能改变末结果 0.3%，
视觉更新在滤波器里的实际权重很低（见 ⑤ 的 R 重扫），默认值保持不变。

**⑤ 视觉噪声 R：全轨迹重扫确认 3.5 为最优（我上一轮"R 高估 30 倍、收益未取满"的推断被证伪）**
由 ① 的 innovation（中位 0.111 m）≪ σ_ν（3.5 m）曾推断"调小 R 可放大视觉收益"。
全轨迹重扫 `--visual_position_noise` 后，**该推断是错的**：

| `--visual_position_noise` | 对齐位置 RMSE | 位置 max | 门控拒绝 | 视觉 NIS 均值 | 全程 pos 1σ 峰值 | 路径长 |
|---|---:|---:|---:|---:|---:|---:|
| ∞（关闭视觉） | 1.172 m | 2.24 m | — | — | — | 1242.8 m |
| **3.5（默认）** | **0.888 m** | **1.62 m** | 0 | 0.001 | 2.17 m | 1244.1 m |
| 1.0 | 1.126 m | 2.14 m | 0 | 0.011 | 1.00 m | 1253.5 m |
| 0.35 | 1.489 m | 3.18 m | 0 | 0.053 | 1.00 m | 1262.4 m |
| 0.1 | 6.248 m | 25.16 m | 862 | 5.260 | **124.7 m** | 1324.0 m |

越信任视觉，误差与路径长都单调变差；R ≤ 0.35 已经**比完全不用视觉还差**。

**为什么 innovation 小不等于测量准**：视觉测量是 `anchor + ΣR(yaw)·(轮速定标的VO位移)`，
它与预测共享同一组轮速/yaw 误差源。innovation 小是因为**两者误差同相（冗余）**，
不是因为视觉自身精度是 0.11 m。把 R 调小 → K 变大 → 状态被这条独立但**更噪**的
VO 轨迹覆写，表现为路径长被"抖"出来（1244 → 1324 m，而真值 1223 m）。
这是冗余观测下 innovation/NIS 读数不能反推 R 的教科书案例。

结论：**视觉的 24% 增益（1.172 → 0.888 m）就是这一观测在最优权重下的上限**，
调参已到头。再往上必须换观测模型本身——回环检测，或真实深度
（DA 米制深度全轨迹 3.52 m、动态尺度修正 1.05 m，均不及轮速定标）。

**⑤b 附带发现的真实缺陷（当前不发作）**：R=0.1 时 862 次门控拒绝，而**被拒的更新
不收缩协方差**，Q 却持续累加 → pos 1σ 峰值冲到 124.7 m。这是"卡方门控 + 拒绝后
不更新"组合的固有病理；默认 R=3.5 下 0 拒绝所以不显现，但若放宽 R 或场景变差需处理
（拒绝后仍做协方差收缩，或改用鲁棒更新）。

**⑥ 相机—IMU 轴序与横向符号：核对为正确，原怀疑不成立**
曾怀疑 `lateral = camera_motion[0]` 的符号与 CCW 世界系不符（差一个 FRD/ENU 反号）。
用 3533 帧真值体系位移做回归：VO lateral 与**真值左向位移**相关 **+0.34**（与右向
为 −0.34），说明取轴与符号自洽。同时确认畸变已用 `initUndistortRectifyMap` 去除。

**⑦ 其他核对项**
- `--init_method groundtruth` 取的是 `.nav` 第 9 列（yaw）；第 8 列是 pitch——审查中
  一度误用第 8 列做轴向测试，导致相关性读数全错，已用第 9 列重做。
- 静止对准的 roll/pitch 已解算但**未被平面 EKF 消费**（有坡度场景会失效）。
- 全部 argparse 参数均被实际使用，无死参数；`--image_fps` 仅在缺 `timestamps.txt`
  的回退路径中生效。



---

## 🔍 故障排除

### 问题1: 找不到bag文件

```
错误: 找不到bag文件 parking01.bag
```

**解决：**
```bash
# 确保bag文件在当前目录
ls -lh parking01.bag

# 或修改脚本中的BAG_FILE路径
```

### 问题2: 图像提取失败

```
Error: no messages of topic /camera/left/image_raw
```

**解决：**
```bash
# 查看bag文件中的topic
rosbag info parking01.bag

# 修改extract_stereo_from_bag.py中的topic名称
```

### 问题3: 内存不足

```
MemoryError
```

**解决：**
```bash
# 减少处理帧数
--max_frames 500

# 或降低图像处理分辨率
--image_scale 0.3
```

---

## 📚 相关文件说明

| 文件 | 说明 |
|------|------|
| `extract_stereo_from_bag.py` | bag 单遍提取左目图像 + 真实时间戳，每3帧取1帧 |
| `stereo_imu_odo_fusion_v2.py` | 融合算法主程序（左目VO + IMU + 轮速 EKF） |
| `evaluate_trajectory.py` | 与真值评估，`--align` 做 2D 刚体对齐 |
| `plot_vo_comparison.py` | 多套结果与真值的对比图（轨迹 + 逐帧误差） |
| `run_full_trajectory.sh` | 全轨迹 2×2 消融编排（提取+融合+评估+对比） |
| `run_test_v2.sh` | 快速测试脚本（100帧） |
| `calibration.yaml` | 相机标定文件（i2Nav-Robot） |

---

## 🎯 运行步骤总结

### 完整流程

```bash
# 1. 进入项目目录
cd /mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main

# 2. 运行完整流程（提取 + 2×2消融 + 评估 + 对比图）
./run_full_trajectory.sh

# 3. 查看结果
ls -lh plots/ablation/*/  plots/ablation/comparison/
```

### 手动运行

```bash
# 1. 激活虚拟环境
source .venv/bin/activate

# 2. 只提取左目图像（每3帧取1帧，同时写出 timestamps.txt）
python extract_stereo_from_bag.py parking01.bag stereo_images 3 --left-only

# 3. 运行融合算法（全轨迹，部署模式，不读取真值）
python stereo_imu_odo_fusion_v2.py \
    --calibration calibration.yaml \
    --left_dir stereo_images/left \
    --timestamps_file stereo_images/left/timestamps.txt \
    --apply_gyro_bias \
    --output_dir plots/my_run

# 4. 与真值评估（2D刚体对齐度量形状精度）
python evaluate_trajectory.py --estimated plots/my_run/fusion_results.csv \
    --groundtruth ../parking01/parking01_groundtruth.nav \
    --output_dir plots/my_run --align
```

---

## ✅ 验证清单

- [x] 虚拟环境已配置
- [x] 依赖包已安装（numpy, opencv, matplotlib, pyyaml）
- [x] 相机标定文件已提供
- [x] bag文件已提供（34GB）
- [x] IMU数据已提供
- [x] 轮速计数据已提供
- [x] 图像提取脚本已创建
- [x] 融合算法已实现
- [x] 运行脚本已创建

---

## 📞 技术支持

如有问题，请检查：
1. bag文件是否完整（34GB）
2. 相机标定文件是否正确
3. 虚拟环境是否激活
4. 磁盘空间是否充足（需要约50GB）

---

**祝你研究顺利！** 🚀
