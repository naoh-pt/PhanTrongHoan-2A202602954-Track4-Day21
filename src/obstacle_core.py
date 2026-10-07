"""Các bước LiDAR dùng chung cho demo CP2 và benchmark CP3 (hệ trục KITTI)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class PipelineResult:
    sampled: o3d.geometry.PointCloud
    ground: o3d.geometry.PointCloud
    obstacles: o3d.geometry.PointCloud
    labels: np.ndarray
    cluster_ids: np.ndarray
    boxes: list[o3d.geometry.AxisAlignedBoundingBox]
    plane: list[float]


def crop_kitti_points(raw: np.ndarray, max_forward: float = 30.0,
                      half_width: float = 15.0) -> tuple[np.ndarray, int]:
    """Bỏ NaN/Inf và lấy ROI x=(0,max_forward], |y|<=half_width, z=[-3,3]."""
    xyz = raw[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    roi = ((xyz[:, 0] > 0) & (xyz[:, 0] <= max_forward)
           & (np.abs(xyz[:, 1]) <= half_width)
           & (xyz[:, 2] >= -3) & (xyz[:, 2] <= 3))
    return xyz[roi], int(len(raw) - finite.sum())


def run_pipeline(cropped: np.ndarray, voxel_size: float = 0.20,
                 ground_threshold: float = 0.20, eps: float = 0.60,
                 min_points: int = 8, seed: int = 0) -> PipelineResult:
    """Voxel -> RANSAC -> DBSCAN -> axis-aligned box; đầu vào đã được crop."""
    if len(cropped) < 3:
        raise ValueError("Vùng quan tâm có quá ít điểm để tách mặt đất")
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cropped)
    sampled = cloud.voxel_down_sample(voxel_size)
    if len(sampled.points) < 3:
        raise ValueError("Voxel size quá lớn: còn quá ít điểm để chạy RANSAC")

    o3d.utility.random.seed(seed)
    plane, ground_ids = sampled.segment_plane(
        distance_threshold=ground_threshold, ransac_n=3, num_iterations=100)
    ground = sampled.select_by_index(ground_ids)
    obstacles = sampled.select_by_index(ground_ids, invert=True)
    labels = np.asarray(obstacles.cluster_dbscan(
        eps=eps, min_points=min_points, print_progress=False))
    cluster_ids = np.unique(labels[labels >= 0])
    boxes = [obstacles.select_by_index(np.flatnonzero(labels == cluster_id))
             .get_axis_aligned_bounding_box() for cluster_id in cluster_ids]
    return PipelineResult(sampled, ground, obstacles, labels, cluster_ids, boxes, plane)
