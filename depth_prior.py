"""Depth Anything V2 metric-depth inference and depth-edge match filtering."""

import sys
from collections import deque
from pathlib import Path
from typing import Deque, Tuple

import cv2
import numpy as np
import torch


MODEL_CONFIG = {
    "encoder": "vits",
    "features": 64,
    "out_channels": [48, 96, 192, 384],
    "max_depth": 20.0,
}


class DynamicDepthScale:
    """Robust online depth-scale calibration using wheel displacement."""

    def __init__(
        self,
        window_size: int = 31,
        min_samples: int = 5,
        min_wheel_distance: float = 0.05,
        min_depth_distance: float = 0.03,
        min_inliers: int = 20,
        min_scale: float = 0.05,
        max_scale: float = 20.0,
    ):
        if window_size < min_samples or min_samples < 1:
            raise ValueError("scale window must be >= a positive minimum sample count")
        if min_wheel_distance <= 0 or min_depth_distance <= 0:
            raise ValueError("minimum scale-calibration distances must be positive")
        if min_inliers < 1 or min_scale <= 0 or max_scale <= min_scale:
            raise ValueError("invalid dynamic depth scale limits")
        self.ratios: Deque[float] = deque(maxlen=window_size)
        self.min_samples = min_samples
        self.min_wheel_distance = min_wheel_distance
        self.min_depth_distance = min_depth_distance
        self.min_inliers = min_inliers
        self.min_scale = min_scale
        self.max_scale = max_scale
        self.scale = 1.0
        self.samples_seen = 0
        self.samples_rejected = 0

    @property
    def calibrated(self) -> bool:
        return len(self.ratios) >= self.min_samples

    def update(
        self,
        depth_distance: float,
        wheel_distance: float,
        inliers: int,
    ) -> bool:
        """Update scale from one valid motion pair; return calibration status."""
        self.samples_seen += 1
        if (
            not np.isfinite([depth_distance, wheel_distance]).all()
            or depth_distance < self.min_depth_distance
            or wheel_distance < self.min_wheel_distance
            or inliers < self.min_inliers
        ):
            self.samples_rejected += 1
            return self.calibrated

        ratio = wheel_distance / depth_distance
        if not np.isfinite(ratio) or not self.min_scale <= ratio <= self.max_scale:
            self.samples_rejected += 1
            return self.calibrated

        self.ratios.append(float(ratio))
        self.scale = float(np.median(np.asarray(self.ratios)))
        return self.calibrated


def depth_discontinuity_mask(
    depth: np.ndarray, threshold: float = 0.08, radius: int = 2
) -> np.ndarray:
    """Mark depth boundaries using the gradient magnitude of log depth."""
    if threshold <= 0:
        raise ValueError("depth edge threshold must be positive")
    if radius < 0:
        raise ValueError("depth edge radius cannot be negative")

    valid = np.isfinite(depth) & (depth > 0)
    safe_depth = np.where(valid, depth, 1.0)
    log_depth = np.log(np.clip(safe_depth, 1e-3, None)).astype(np.float32)
    dx = cv2.Sobel(log_depth, cv2.CV_32F, 1, 0, ksize=3, scale=0.125)
    dy = cv2.Sobel(log_depth, cv2.CV_32F, 0, 1, ksize=3, scale=0.125)
    edges = (~valid) | (np.hypot(dx, dy) >= threshold)

    if radius:
        size = 2 * radius + 1
        kernel = np.ones((size, size), dtype=np.uint8)
        edges = cv2.dilate(edges.astype(np.uint8), kernel).astype(bool)
    return edges


def estimate_metric_pose(
    points0: np.ndarray,
    points1: np.ndarray,
    depth0: np.ndarray,
    camera_matrix: np.ndarray,
    min_depth: float = 0.2,
    max_depth: float = 20.0,
    reprojection_error: float = 3.0,
    min_inliers: int = 8,
) -> Tuple[np.ndarray, np.ndarray, int]:
    """Estimate metric motion from 3D points in frame 0 to 2D matches in frame 1."""
    xy0 = np.rint(points0).astype(np.int32)
    valid = (
        (xy0[:, 0] >= 0)
        & (xy0[:, 0] < depth0.shape[1])
        & (xy0[:, 1] >= 0)
        & (xy0[:, 1] < depth0.shape[0])
    )
    valid_indices = np.flatnonzero(valid)
    z = depth0[xy0[valid, 1], xy0[valid, 0]]
    valid_depth = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    valid_indices = valid_indices[valid_depth]
    if len(valid_indices) < min_inliers:
        raise ValueError("not enough matches with valid metric depth")

    pixels0 = points0[valid_indices].astype(np.float64)
    pixels1 = points1[valid_indices].astype(np.float64)
    z = depth0[
        np.rint(pixels0[:, 1]).astype(np.int32),
        np.rint(pixels0[:, 0]).astype(np.int32),
    ].astype(np.float64)
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    points3d = np.column_stack(
        ((pixels0[:, 0] - cx) * z / fx, (pixels0[:, 1] - cy) * z / fy, z)
    ).astype(np.float32)
    success, rotation_vector, translation, inlier_indices = cv2.solvePnPRansac(
        points3d,
        pixels1.astype(np.float32),
        camera_matrix.astype(np.float64),
        None,
        iterationsCount=1000,
        reprojectionError=reprojection_error,
        confidence=0.999,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if (
        not success
        or inlier_indices is None
        or len(inlier_indices) < min_inliers
    ):
        raise ValueError("metric-depth PnP did not find enough inliers")

    inlier_indices = inlier_indices.reshape(-1)
    success, rotation_vector, translation = cv2.solvePnP(
        points3d[inlier_indices],
        pixels1[inlier_indices].astype(np.float32),
        camera_matrix.astype(np.float64),
        None,
        rotation_vector,
        translation,
        True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise ValueError("metric-depth PnP refinement failed")

    rotation, _ = cv2.Rodrigues(rotation_vector)
    translation = translation.reshape(3)
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ValueError("metric-depth PnP returned non-finite pose")
    return rotation, translation, len(inlier_indices)


class MetricDepthEdgeFilter:
    """Use metric Depth Anything V2 to remove matches near depth boundaries."""

    def __init__(
        self,
        checkpoint: str,
        input_size: int = 518,
        edge_threshold: float = 0.08,
        edge_radius: int = 2,
    ):
        checkpoint_path = Path(checkpoint)
        if not checkpoint_path.is_file():
            raise FileNotFoundError(
                f"Depth Anything checkpoint not found: {checkpoint_path}"
            )
        if input_size < 14:
            raise ValueError("Depth Anything input size must be at least 14 pixels")
        if edge_threshold <= 0:
            raise ValueError("depth edge threshold must be positive")
        if edge_radius < 0:
            raise ValueError("depth edge radius cannot be negative")

        model_root = (
            Path(__file__).resolve().parent
            / "Depth-Anything-V2-main"
            / "metric_depth"
        )
        if not model_root.is_dir():
            raise FileNotFoundError(
                f"Depth Anything metric model source not found: {model_root}"
            )
        if str(model_root) not in sys.path:
            sys.path.insert(0, str(model_root))

        from depth_anything_v2.dpt import DepthAnythingV2

        self.input_size = input_size
        self.edge_threshold = edge_threshold
        self.edge_radius = edge_radius
        self.checkpoint_path = checkpoint_path
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.model = DepthAnythingV2(**MODEL_CONFIG)
        state_dict = torch.load(
            str(checkpoint_path), map_location="cpu", weights_only=True
        )
        self.model.load_state_dict(state_dict)
        self.model.to(self.device).eval()

        self.matches_seen = 0
        self.matches_rejected = 0
        print(
            "Depth Anything V2 深度边缘筛选已启用: "
            f"{checkpoint_path} (设备: {self.device}, 输入尺寸: {input_size}, "
            f"边缘阈值: {edge_threshold}, 膨胀半径: {edge_radius}px)"
        )

    def infer_depth(self, image: np.ndarray) -> np.ndarray:
        return self.model.infer_image(image, input_size=self.input_size)

    def edge_mask(self, depth: np.ndarray) -> np.ndarray:
        return depth_discontinuity_mask(
            depth, threshold=self.edge_threshold, radius=self.edge_radius
        )

    def filter_matches(
        self,
        matches: np.ndarray,
        points0: np.ndarray,
        points1: np.ndarray,
        edge0: np.ndarray,
        edge1: np.ndarray,
    ) -> Tuple[np.ndarray, int]:
        self.matches_seen += len(matches)
        if not len(matches):
            return matches, 0

        height0, width0 = edge0.shape
        height1, width1 = edge1.shape
        xy0 = np.rint(points0).astype(np.int32)
        xy1 = np.rint(points1).astype(np.int32)
        inside = (
            (xy0[:, 0] >= 0)
            & (xy0[:, 0] < width0)
            & (xy0[:, 1] >= 0)
            & (xy0[:, 1] < height0)
            & (xy1[:, 0] >= 0)
            & (xy1[:, 0] < width1)
            & (xy1[:, 1] >= 0)
            & (xy1[:, 1] < height1)
        )
        keep = inside.copy()
        valid_indices = np.flatnonzero(inside)
        keep[valid_indices] &= ~(
            edge0[xy0[valid_indices, 1], xy0[valid_indices, 0]]
            | edge1[xy1[valid_indices, 1], xy1[valid_indices, 0]]
        )

        rejected = int(len(matches) - np.count_nonzero(keep))
        self.matches_rejected += rejected
        return matches[keep], rejected
