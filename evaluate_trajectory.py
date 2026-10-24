#!/mnt/fccedaf2-554d-47e5-8831-30c12ee6e08d/mechine/GPS_IMU_Fusion_EKF-main/.venv/bin/python3
"""
评估融合定位结果
计算RMSE并生成轨迹对比图
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import argparse
import math
from scipy.interpolate import interp1d


def load_fusion_results(csv_file):
    """加载融合结果"""
    data = np.genfromtxt(csv_file, delimiter=',', skip_header=1)
    time = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    yaw = data[:, 3]
    speed = data[:, 4]
    return time, x, y, yaw, speed


def load_groundtruth(nav_file):
    """加载真值数据"""
    data = np.genfromtxt(nav_file)
    time = data[:, 0]
    x = data[:, 1]
    y = data[:, 2]
    z = data[:, 3]
    vx = data[:, 4]
    vy = data[:, 5]
    vz = data[:, 6]
    roll = data[:, 7]
    pitch = data[:, 8]
    yaw = data[:, 9]
    return time, x, y, z, vx, vy, vz, roll, pitch, yaw


def align_trajectories(time_est, time_gt, nav_file):
    """Time alignment: interpolate GT to estimated timestamps."""
    t_min = max(time_est[0], time_gt[0])
    t_max = min(time_est[-1], time_gt[-1])
    
    mask_est = (time_est >= t_min) & (time_est <= t_max)
    mask_gt = (time_gt >= t_min) & (time_gt <= t_max)
    
    idx_est = np.where(mask_est)[0]
    time_est_aligned = time_est[idx_est]
    
    gt_data = np.genfromtxt(nav_file)
    f_x = interp1d(time_gt[mask_gt], gt_data[mask_gt, 1],
                   bounds_error=False, fill_value='extrapolate')
    f_y = interp1d(time_gt[mask_gt], gt_data[mask_gt, 2],
                   bounds_error=False, fill_value='extrapolate')
    # GT yaw is in degrees (col 9); convert to radians to match estimate
    f_yaw = interp1d(time_gt[mask_gt], np.radians(gt_data[mask_gt, 9]),
                     bounds_error=False, fill_value='extrapolate')

    x_gt_aligned = f_x(time_est_aligned)
    y_gt_aligned = f_y(time_est_aligned)
    yaw_gt_aligned = f_yaw(time_est_aligned)
    # nav 文件的 yaw 是 0~360° 折绕值；线性插值会在跨 0°/360° 处产生
    # 假跳变（359.9°→0.1° 被插成 180°），必须先展开再参与比较。
    yaw_gt_aligned = np.unwrap(yaw_gt_aligned)

    return time_est_aligned, x_gt_aligned, y_gt_aligned, yaw_gt_aligned, idx_est


def rigid_align_2d(x_est, y_est, yaw_est, x_gt, y_gt, yaw_gt):
    """Align an odometry-frame trajectory to ground truth with a 2D rigid
    transform (rotation + translation, no scale), using orthogonal Procrustes
    on corresponding points. Absolute heading is unobservable without GNSS/a
    magnetometer, so the estimate starts in the vehicle odometry frame; this
    fair relative-odometry alignment measures trajectory shape accuracy.

    Returns aligned estimate x, y, yaw and the applied heading offset (rad).
    位置用 Procrustes 旋转+平移对齐；偏航单独减去其误差圆均值（见下）。
    """
    est = np.stack([x_est, y_est], axis=1)
    gt = np.stack([x_gt, y_gt], axis=1)
    mu_e = est.mean(axis=0)
    mu_g = gt.mean(axis=0)
    E = est - mu_e
    G = gt - mu_g
    H = E.T @ G
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    Rm = Vt.T @ np.diag([1.0, d]) @ U.T          # rotation est -> gt
    t = mu_g - Rm @ mu_e                          # translation
    est_a = (Rm @ est.T).T + t
    # Heading offset implied by the rotation
    theta = math.atan2(Rm[1, 0], Rm[0, 0])
    # 偏航不用位置拟合出的旋转角来对齐：那会让"偏航精度"被位置形状带着走
    # （实测同一份 yaw 状态，仅因位置轨迹不同，RMSE 就从 1.91° 变 6.28°）。
    # 改为减去偏航误差自身的圆均值，即最优常值航向偏置，度量真正的相对航向漂移。
    dpsi = np.asarray(yaw_gt) - np.asarray(yaw_est)
    psi0 = math.atan2(float(np.mean(np.sin(dpsi))), float(np.mean(np.cos(dpsi))))
    yaw_a = np.array([(yaw + psi0 + np.pi) % (2 * np.pi) - np.pi for yaw in yaw_est])
    return est_a[:, 0], est_a[:, 1], yaw_a, psi0


def calculate_errors(x_est, y_est, yaw_est, x_gt, y_gt, yaw_gt):
    """计算误差"""
    # 位置误差
    pos_error = np.sqrt((x_est - x_gt)**2 + (y_est - y_gt)**2)
    
    # 偏航角误差（处理角度跳变）
    yaw_error = np.abs(yaw_est - yaw_gt)
    yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))  # 归一化到[-pi, pi]
    yaw_error_deg = np.degrees(np.abs(yaw_error))
    
    # RMSE
    rmse_pos = np.sqrt(np.mean(pos_error**2))
    rmse_yaw = np.sqrt(np.mean(yaw_error_deg**2))
    
    # 平均误差
    mean_pos = np.mean(pos_error)
    mean_yaw = np.mean(yaw_error_deg)
    
    # 最大误差
    max_pos = np.max(pos_error)
    max_yaw = np.max(yaw_error_deg)
    
    return {
        'pos_error': pos_error,
        'yaw_error': yaw_error_deg,
        'rmse_pos': rmse_pos,
        'rmse_yaw': rmse_yaw,
        'mean_pos': mean_pos,
        'mean_yaw': mean_yaw,
        'max_pos': max_pos,
        'max_yaw': max_yaw
    }


def plot_trajectory_comparison(time_est, x_est, y_est, x_gt, y_gt, output_dir):
    """绘制轨迹对比图"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # 1. 2D轨迹对比
    ax = axes[0, 0]
    ax.plot(x_gt, y_gt, 'b-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(x_est, y_est, 'r-', linewidth=2, label='Estimated', alpha=0.7)
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_title('2D Trajectory Comparison', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    # 2. X坐标对比
    ax = axes[0, 1]
    duration = time_est - time_est[0]
    ax.plot(duration, x_gt, 'b-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(duration, x_est, 'r-', linewidth=2, label='Estimated', alpha=0.7)
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('X [m]', fontsize=12)
    ax.set_title('X Coordinate', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # 3. Y坐标对比
    ax = axes[1, 0]
    ax.plot(duration, y_gt, 'b-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(duration, y_est, 'r-', linewidth=2, label='Estimated', alpha=0.7)
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_title('Y Coordinate', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # 4. 3D轨迹对比
    ax = axes[1, 1]
    ax = fig.add_subplot(224, projection='3d')
    ax.plot(x_gt, y_gt, duration, 'b-', linewidth=2, label='Ground Truth', alpha=0.7)
    ax.plot(x_est, y_est, duration, 'r-', linewidth=2, label='Estimated', alpha=0.7)
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Y [m]', fontsize=12)
    ax.set_zlabel('Time [s]', fontsize=12)
    ax.set_title('3D Trajectory', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'trajectory_comparison.png', dpi=150, bbox_inches='tight')
    print(f"轨迹对比图已保存: {output_dir / 'trajectory_comparison.png'}")


def plot_error_analysis(time_est, errors, output_dir):
    """绘制误差分析图"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    duration = time_est - time_est[0]
    
    # 1. 位置误差随时间变化
    ax = axes[0, 0]
    ax.plot(duration, errors['pos_error'], 'r-', linewidth=1.5, alpha=0.7)
    ax.axhline(y=errors['rmse_pos'], color='k', linestyle='--', linewidth=2, 
               label=f'RMSE: {errors["rmse_pos"]:.3f} m')
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Position Error [m]', fontsize=12)
    ax.set_title('Position Error Over Time', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # 2. 偏航角误差随时间变化
    ax = axes[0, 1]
    ax.plot(duration, errors['yaw_error'], 'b-', linewidth=1.5, alpha=0.7)
    ax.axhline(y=errors['rmse_yaw'], color='k', linestyle='--', linewidth=2,
               label=f'RMSE: {errors["rmse_yaw"]:.3f} deg')
    ax.set_xlabel('Time [s]', fontsize=12)
    ax.set_ylabel('Yaw Error [deg]', fontsize=12)
    ax.set_title('Yaw Error Over Time', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # 3. 位置误差直方图
    ax = axes[1, 0]
    ax.hist(errors['pos_error'], bins=50, color='r', alpha=0.7, edgecolor='black')
    ax.axvline(x=errors['mean_pos'], color='k', linestyle='--', linewidth=2,
               label=f'Mean: {errors["mean_pos"]:.3f} m')
    ax.axvline(x=errors['rmse_pos'], color='b', linestyle='--', linewidth=2,
               label=f'RMSE: {errors["rmse_pos"]:.3f} m')
    ax.set_xlabel('Position Error [m]', fontsize=12)
    ax.set_ylabel('Frequency', fontsize=12)
    ax.set_title('Position Error Distribution', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    # 4. 偏航角误差直方图
    ax = axes[1, 1]
    ax.hist(errors['yaw_error'], bins=50, color='b', alpha=0.7, edgecolor='black')
    ax.axvline(x=errors['mean_yaw'], color='k', linestyle='--', linewidth=2,
               label=f'Mean: {errors["mean_yaw"]:.3f} deg')
    ax.axvline(x=errors['rmse_yaw'], color='r', linestyle='--', linewidth=2,
               label=f'RMSE: {errors["rmse_yaw"]:.3f} deg')
    ax.set_xlabel('Yaw Error [deg]', fontsize=12)
    ax.set_ylabel('Frequency', fontsize=12)
    ax.set_title('Yaw Error Distribution', fontsize=14, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'error_analysis.png', dpi=150, bbox_inches='tight')
    print(f"误差分析图已保存: {output_dir / 'error_analysis.png'}")


def save_error_report(errors, output_dir):
    """保存误差报告"""
    report_file = output_dir / 'error_report.txt'
    
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write("=" * 60 + "\n")
        f.write("融合定位误差评估报告\n")
        f.write("=" * 60 + "\n\n")
        
        f.write("位置误差统计:\n")
        f.write(f"  RMSE:      {errors['rmse_pos']:.4f} m\n")
        f.write(f"  Mean:      {errors['mean_pos']:.4f} m\n")
        f.write(f"  Max:       {errors['max_pos']:.4f} m\n")
        f.write("\n")
        
        f.write("偏航角误差统计:\n")
        f.write(f"  RMSE:      {errors['rmse_yaw']:.4f} deg\n")
        f.write(f"  Mean:      {errors['mean_yaw']:.4f} deg\n")
        f.write(f"  Max:       {errors['max_yaw']:.4f} deg\n")
        f.write("\n")
        
        f.write("=" * 60 + "\n")
    
    print(f"误差报告已保存: {report_file}")


def main():
    parser = argparse.ArgumentParser(description='评估融合定位结果')
    parser.add_argument('--estimated', type=str,
                        default='plots/test_run/fusion_results.csv',
                        help='融合结果CSV文件')
    parser.add_argument('--groundtruth', type=str,
                        default='../parking01/parking01_groundtruth.nav',
                        help='真值数据文件')
    parser.add_argument('--output_dir', type=str,
                        default='plots/test_run',
                        help='输出目录')
    parser.add_argument('--align', action='store_true',
                        help='对估计轨迹做2D刚体对齐(旋转+平移)后再计算误差。'
                             '用于无绝对航向参考(IMU静止对准, yaw=0里程计系)的公平评估')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("融合定位误差评估")
    print("=" * 60)
    print()
    
    # 加载数据
    print("加载融合结果...")
    time_est, x_est, y_est, yaw_est, speed_est = load_fusion_results(args.estimated)
    print(f"  融合结果: {len(time_est)} 个时间点")
    print(f"  时间范围: {time_est[0]:.2f} - {time_est[-1]:.2f} s")
    
    print("\n加载真值数据...")
    time_gt, x_gt, y_gt, z_gt, vx_gt, vy_gt, vz_gt, roll_gt, pitch_gt, yaw_gt = \
        load_groundtruth(args.groundtruth)
    print(f"  真值数据: {len(time_gt)} 个时间点")
    print(f"  时间范围: {time_gt[0]:.2f} - {time_gt[-1]:.2f} s")
    
    # 时间对齐
    print("\n时间对齐...")
    time_aligned, x_gt_aligned, y_gt_aligned, yaw_gt_aligned, idx_est = \
        align_trajectories(time_est, time_gt, args.groundtruth)
    print(f"  对齐后: {len(time_aligned)} 个时间点")
    
    # 计算误差
    print("\n计算误差...")
    x_est_aligned = x_est[idx_est]
    y_est_aligned = y_est[idx_est]
    yaw_est_aligned = yaw_est[idx_est]

    if args.align:
        print("\n2D刚体对齐（旋转+平移，无尺度）...")
        x_est_aligned, y_est_aligned, yaw_est_aligned, theta = rigid_align_2d(
            x_est_aligned, y_est_aligned, yaw_est_aligned,
            x_gt_aligned, y_gt_aligned, yaw_gt_aligned)
        print(f"  对齐航向偏移: {np.degrees(theta):.2f}°")
    
    errors = calculate_errors(x_est_aligned, y_est_aligned, yaw_est_aligned,
                              x_gt_aligned, y_gt_aligned, yaw_gt_aligned)
    
    # 打印误差统计
    print("\n" + "=" * 60)
    print("误差统计")
    print("=" * 60)
    print(f"\n位置误差:")
    print(f"  RMSE:  {errors['rmse_pos']:.4f} m")
    print(f"  Mean:  {errors['mean_pos']:.4f} m")
    print(f"  Max:   {errors['max_pos']:.4f} m")
    
    print(f"\n偏航角误差:")
    print(f"  RMSE:  {errors['rmse_yaw']:.4f} deg")
    print(f"  Mean:  {errors['mean_yaw']:.4f} deg")
    print(f"  Max:   {errors['max_yaw']:.4f} deg")
    
    # 生成图表
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("\n生成轨迹对比图...")
    plot_trajectory_comparison(time_aligned, x_est_aligned, y_est_aligned,
                               x_gt_aligned, y_gt_aligned, output_dir)
    
    print("\n生成误差分析图...")
    plot_error_analysis(time_aligned, errors, output_dir)
    
    print("\n保存误差报告...")
    save_error_report(errors, output_dir)
    
    print("\n" + "=" * 60)
    print("评估完成！")
    print("=" * 60)
    print(f"\n结果已保存到: {output_dir}")
    print("  - trajectory_comparison.png")
    print("  - error_analysis.png")
    print("  - error_report.txt")


if __name__ == '__main__':
    main()
