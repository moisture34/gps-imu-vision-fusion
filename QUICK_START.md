# 🚀 快速开始指南

## 你的研究目标
**室内无GNSS移动机器人IMU与轮式里程计融合定位方法研究**

## 推荐运行顺序

### 第1步：快速测试（2分钟）
验证系统是否正常工作

```bash
cd /mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main
./run_test.sh
```

**这会做什么：**
- 处理100帧图像（采样后约33帧）
- 生成测试轨迹图
- 验证所有依赖正常

**查看结果：**
```bash
cat plots/test_run/metrics.txt
```

---

### 第2步：完整运行（5-10分钟）
处理所有数据

```bash
./run_fusion.sh
```

**这会做什么：**
- 处理1200帧图像（采样后约400帧）
- 生成完整的融合结果
- 输出性能指标

**查看结果：**
```bash
# 查看性能指标
cat plots/indoor_fusion/metrics.txt

# 查看轨迹图
ls -lh plots/indoor_fusion/*.png
```

---

## 📊 输出文件说明

运行完成后，你会得到：

```
plots/indoor_fusion/
├── fusion_results.csv       # 融合结果数据（可用于进一步分析）
├── fusion_trajectory.png    # 轨迹对比图（真值 vs 估计）
├── fusion_error.png         # 位置误差随时间变化
└── metrics.txt              # 性能指标汇总
```

---

## 🎯 核心算法

你的系统使用**三传感器融合**：

```
单目相机 (10Hz) ──┐
                  │
IMU (200Hz) ──────┼──→ 扩展卡尔曼滤波 ──→ 融合定位结果
                  │       (EKF)
轮速计 (50Hz) ────┘
```

**关键特性：**
- ✅ 每3帧取1帧（保持时间连续性）
- ✅ 单目视觉里程计（本质矩阵分解）
- ✅ IMU提供角速度预测
- ✅ 轮速计提供速度约束
- ✅ EKF最优融合

---

## 🔬 研究建议

### 1. 参数调优实验
尝试不同的噪声参数：

```bash
# 增加视觉信任度
python stereo_imu_odo_fusion.py \
    --visual-position-noise 0.30 \
    --visual-yaw-noise 0.08 \
    ...

# 增加轮速计信任度
python stereo_imu_odo_fusion.py \
    --wheel-speed-noise 0.10 \
    ...
```

### 2. 消融实验
对比不同传感器组合：

| 实验 | 配置 | 目的 |
|------|------|------|
| A | 仅视觉 | 评估视觉里程计性能 |
| B | 仅IMU+轮速计 | 评估传统方法 |
| C | 视觉+IMU+轮速计 | 评估融合效果 |

### 3. 采样策略实验
修改采样率（当前每3帧取1帧）：

```python
# 在 stereo_imu_odo_fusion.py 第 237 行
left_files = [f for i, f in enumerate(left_files) if i % 2 == 0]  # 每2帧
left_files = [f for i, f in enumerate(left_files) if i % 5 == 0]  # 每5帧
```

---

## 📈 预期结果

基于 parking01 数据集：

| 指标 | 预期范围 | 说明 |
|------|---------|------|
| 中位数误差 | 0.3 - 0.8 m | 典型定位精度 |
| 平均误差 | 0.5 - 1.2 m | 整体性能 |
| 95%分位误差 | 1.0 - 2.5 m | 最差情况 |
| 视觉成功率 | 70% - 90% | 特征匹配成功率 |

---

## 📚 详细文档

- **RUN_GUIDE.md** - 完整运行指南和参数说明
- **MONO_VO_CHANGES.md** - 算法修改详情
- **PROJECT_SETUP.md** - 项目设置报告
- **ACTIVATE_VENV.md** - 虚拟环境说明

---

## ❓ 常见问题

**Q: 运行失败怎么办？**
A: 先运行 `./run_test.sh` 进行快速测试

**Q: 如何修改采样率？**
A: 编辑 `stereo_imu_odo_fusion.py` 第 237 行

**Q: 如何只使用IMU+轮速计（不用视觉）？**
A: 我可以帮你创建一个专门的脚本，联系我说明

**Q: 运行时间太长？**
A: 减少 `--max-frames` 或降低 `--image-scale`

---

## 🎓 下一步

1. ✅ 运行快速测试
2. ✅ 运行完整融合
3. ✅ 分析结果和性能指标
4. ✅ 进行参数调优实验
5. ✅ 撰写研究报告

---

**祝你的研究取得成功！** 🚀

如有问题，请查看详细文档 `RUN_GUIDE.md`
