"""Evaluate visual odometry frame-to-frame translation against ground truth."""

import argparse
from pathlib import Path

import numpy as np


def evaluate(vo_file: Path, groundtruth_file: Path, output_dir: Path) -> dict:
    vo = np.genfromtxt(vo_file, delimiter=",", names=True)
    groundtruth = np.loadtxt(groundtruth_file)
    if vo.ndim == 0 or len(vo) < 2:
        raise ValueError("VO motion file must contain at least two frames")
    if groundtruth.ndim != 2 or groundtruth.shape[1] < 3:
        raise ValueError("ground-truth file must contain time, x, and y columns")

    time = np.atleast_1d(vo["time"])
    success = np.atleast_1d(vo["success"]).astype(bool)
    forward = np.atleast_1d(vo["forward_m"])
    lateral = np.atleast_1d(vo["lateral_m"])
    estimated_distance = np.hypot(forward, lateral)
    raw_forward = np.atleast_1d(vo["raw_forward_m"])
    raw_lateral = np.atleast_1d(vo["raw_lateral_m"])
    raw_distance = np.hypot(raw_forward, raw_lateral)

    gt_start = groundtruth[0, 0]
    gt_end = groundtruth[-1, 0]
    time_valid = (
        (time[:-1] >= gt_start)
        & (time[1:] >= gt_start)
        & (time[1:] <= gt_end)
    )
    gt_x = np.interp(time, groundtruth[:, 0], groundtruth[:, 1])
    gt_y = np.interp(time, groundtruth[:, 0], groundtruth[:, 2])
    reference_distance = np.hypot(np.diff(gt_x), np.diff(gt_y))
    valid = (
        success[1:]
        & time_valid
        & np.isfinite(estimated_distance[1:])
    )
    if not np.any(valid):
        raise ValueError("no valid VO frame translations to evaluate")

    estimated = estimated_distance[1:][valid]
    raw_estimated = raw_distance[1:][valid]
    reference = reference_distance[valid]
    error = estimated - reference
    raw_error = raw_estimated - reference
    scale_factor = np.atleast_1d(vo["depth_scale_factor"])[1:][valid]
    metrics = {
        "valid_frames": int(np.count_nonzero(valid)),
        "vo_success_rate": float(np.mean(success[1:])),
        "distance_rmse_m": float(np.sqrt(np.mean(error**2))),
        "distance_mae_m": float(np.mean(np.abs(error))),
        "distance_bias_m": float(np.mean(error)),
        "raw_distance_rmse_m": float(np.sqrt(np.mean(raw_error**2))),
        "raw_distance_mae_m": float(np.mean(np.abs(raw_error))),
        "raw_distance_bias_m": float(np.mean(raw_error)),
        "scale_factor_median": float(np.median(scale_factor)),
        "scale_factor_min": float(np.min(scale_factor)),
        "scale_factor_max": float(np.max(scale_factor)),
        "distance_reference_mean_m": float(np.mean(reference)),
        "distance_estimated_mean_m": float(np.mean(estimated)),
        "raw_distance_estimated_mean_m": float(np.mean(raw_estimated)),
    }

    output_dir.mkdir(parents=True, exist_ok=True)
    errors = np.column_stack(
        (
            time[1:][valid],
            reference,
            raw_estimated,
            estimated,
            error,
            scale_factor,
        )
    )
    np.savetxt(
        output_dir / "vo_distance_errors.csv",
        errors,
        delimiter=",",
        header=(
            "time,groundtruth_distance_m,raw_vo_distance_m,"
            "corrected_vo_distance_m,corrected_error_m,depth_scale_factor"
        ),
        comments="",
    )
    (output_dir / "vo_distance_report.txt").write_text(
        "Frame-to-frame horizontal translation distance evaluation\n"
        f"VO file: {vo_file}\n"
        f"Ground truth: {groundtruth_file}\n"
        f"Valid frames: {metrics['valid_frames']}\n"
        f"VO success rate: {metrics['vo_success_rate']:.4%}\n"
        f"Reference mean distance: {metrics['distance_reference_mean_m']:.6f} m\n"
        f"Raw VO mean distance: {metrics['raw_distance_estimated_mean_m']:.6f} m\n"
        f"Corrected VO mean distance: {metrics['distance_estimated_mean_m']:.6f} m\n"
        f"Raw distance RMSE: {metrics['raw_distance_rmse_m']:.6f} m\n"
        f"Corrected distance RMSE: {metrics['distance_rmse_m']:.6f} m\n"
        f"Corrected distance MAE: {metrics['distance_mae_m']:.6f} m\n"
        f"Corrected distance bias: {metrics['distance_bias_m']:.6f} m\n"
        f"Scale factor median/min/max: {metrics['scale_factor_median']:.6f} / "
        f"{metrics['scale_factor_min']:.6f} / {metrics['scale_factor_max']:.6f}\n"
    )
    return metrics


def main():
    parser = argparse.ArgumentParser(
        description="Evaluate VO frame-to-frame metric translation distances"
    )
    parser.add_argument("--vo", required=True, help="vo_motion_results.csv")
    parser.add_argument("--groundtruth", required=True, help="ground-truth NAV file")
    parser.add_argument("--output_dir", required=True)
    args = parser.parse_args()

    metrics = evaluate(Path(args.vo), Path(args.groundtruth), Path(args.output_dir))
    print("VO translation distance evaluation")
    for name, value in metrics.items():
        if name.endswith("_m"):
            print(f"  {name}: {value:.6f} m")
        elif name == "vo_success_rate":
            print(f"  {name}: {value:.2%}")
        else:
            print(f"  {name}: {value}")


if __name__ == "__main__":
    main()
