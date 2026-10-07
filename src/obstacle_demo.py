"""CP2 demo: phát hiện vật cản LiDAR bằng voxel, RANSAC và DBSCAN.

Chạy từ gốc repo: python -m src.obstacle_demo --data-root data/kitti_mini --frame 000011
Hệ trục áp dụng cho KITTI: x hướng trước xe, y sang trái, z hướng lên.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import tempfile

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "topic_d_matplotlib"))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import open3d as o3d

from starter.datasets import dataset_type, load_points


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default="data/kitti_mini")
    parser.add_argument("--frame", default="000011")
    parser.add_argument("--voxel-size", type=float, default=0.20, help="Kích thước voxel (m)")
    parser.add_argument("--ground-threshold", type=float, default=0.20,
                        help="Ngưỡng khoảng cách tới mặt phẳng RANSAC (m)")
    parser.add_argument("--eps", type=float, default=0.60, help="Bán kính DBSCAN (m)")
    parser.add_argument("--min-points", type=int, default=8, help="Số điểm tối thiểu của DBSCAN")
    parser.add_argument("--max-forward", type=float, default=30.0, help="Giới hạn x phía trước (m)")
    parser.add_argument("--half-width", type=float, default=15.0, help="Giới hạn |y| (m)")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path,
                        default=Path("results/figures/obstacle_demo_000011.png"))
    args = parser.parse_args()
    if args.voxel_size <= 0 or args.ground_threshold <= 0 or args.eps <= 0:
        parser.error("voxel-size, ground-threshold và eps phải > 0")
    if args.min_points < 1 or args.max_forward <= 0 or args.half_width <= 0:
        parser.error("min-points >= 1, max-forward > 0 và half-width > 0")
    return args


def make_cloud(xyz: np.ndarray) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    return cloud


def setup_bev(ax, title: str, max_forward: float, half_width: float) -> None:
    ax.set_title(title)
    ax.set_xlim(0, max_forward)
    ax.set_ylim(-half_width, half_width)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x: phía trước (m)")
    ax.set_ylabel("y: sang trái (m)")
    ax.grid(alpha=0.15)


def scatter_xy(ax, xyz: np.ndarray, *, color, size: float = 0.35) -> None:
    if len(xyz):
        ax.scatter(xyz[:, 0], xyz[:, 1], s=size, c=color, linewidths=0,
                   rasterized=True)


def main() -> None:
    # Windows có thể mặc định dùng cp1252, không in được --help tiếng Việt.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
    args = parse_args()
    if dataset_type(args.data_root) != "kitti":
        raise ValueError("Demo này dùng hệ trục KITTI; hãy chọn data/kitti_mini hoặc data/synthetic")

    raw = load_points(args.data_root, args.frame)
    xyz = raw[:, :3]
    finite = np.isfinite(xyz).all(axis=1)
    xyz = xyz[finite]
    roi = ((xyz[:, 0] > 0) & (xyz[:, 0] <= args.max_forward)
           & (np.abs(xyz[:, 1]) <= args.half_width)
           & (xyz[:, 2] >= -3) & (xyz[:, 2] <= 3))
    cropped = xyz[roi]
    if len(cropped) < 3:
        raise ValueError("Vùng quan tâm có quá ít điểm để tách mặt đất")

    cloud = make_cloud(cropped)
    sampled = cloud.voxel_down_sample(args.voxel_size)
    if len(sampled.points) < 3:
        raise ValueError("Voxel size quá lớn: còn quá ít điểm để chạy RANSAC")

    o3d.utility.random.seed(args.seed)
    plane, ground_ids = sampled.segment_plane(
        distance_threshold=args.ground_threshold, ransac_n=3, num_iterations=100)
    ground = sampled.select_by_index(ground_ids)
    obstacles = sampled.select_by_index(ground_ids, invert=True)
    labels = np.asarray(obstacles.cluster_dbscan(
        eps=args.eps, min_points=args.min_points, print_progress=False))
    cluster_ids = np.unique(labels[labels >= 0])

    sampled_xyz = np.asarray(sampled.points)
    ground_xyz = np.asarray(ground.points)
    obstacle_xyz = np.asarray(obstacles.points)
    fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
    for ax, title in zip(axes.flat, (
        f"1. Điểm gốc trong ROI: {len(cropped):,}",
        f"2. Sau voxel {args.voxel_size:g} m: {len(sampled_xyz):,}",
        f"3. Đất: {len(ground_xyz):,} | còn lại: {len(obstacle_xyz):,}",
        f"4. DBSCAN: {len(cluster_ids)} cụm + bounding box",
    )):
        setup_bev(ax, title, args.max_forward, args.half_width)

    scatter_xy(axes[0, 0], cropped, color="#334155")
    scatter_xy(axes[0, 1], sampled_xyz, color="#334155", size=0.8)
    scatter_xy(axes[1, 0], ground_xyz, color="#94a3b8", size=0.8)
    scatter_xy(axes[1, 0], obstacle_xyz, color="#ea580c", size=0.8)
    scatter_xy(axes[1, 1], obstacle_xyz[labels < 0], color="#cbd5e1", size=0.6)
    colors = plt.get_cmap("tab20", max(len(cluster_ids), 1))
    for color_index, cluster_id in enumerate(cluster_ids):
        cluster_xyz = obstacle_xyz[labels == cluster_id]
        color = colors(color_index)
        scatter_xy(axes[1, 1], cluster_xyz, color=[color], size=1.5)
        box = obstacles.select_by_index(np.flatnonzero(labels == cluster_id)).get_axis_aligned_bounding_box()
        axes[1, 1].add_patch(Rectangle(
            box.min_bound[:2], *(box.max_bound[:2] - box.min_bound[:2]),
            fill=False, edgecolor=color, linewidth=0.9))

    fig.suptitle(f"Topic D | KITTI {args.frame} | BEV (nhìn từ trên xuống)")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=160)
    plt.close(fig)
    print(f"frame={args.frame} raw={len(raw)} invalid={len(raw) - finite.sum()} "
          f"roi={len(cropped)} voxel={len(sampled_xyz)} ground={len(ground_xyz)} "
          f"non_ground={len(obstacle_xyz)} clusters={len(cluster_ids)} "
          f"plane_normal={np.asarray(plane[:3]).round(3).tolist()}")
    print(f"figure={args.out}")


if __name__ == "__main__":
    main()
