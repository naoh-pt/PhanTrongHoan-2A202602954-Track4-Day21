"""CP4: minh họa DBSCAN gộp các cặp Pedestrian ở KITTI 000043."""

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

from starter.datasets import load_points
from starter.kitti_io import frame_paths, load_calib, load_labels
from src.obstacle_core import crop_kitti_points, run_pipeline


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default="data/kitti_mini")
    parser.add_argument("--frame", default="000043")
    parser.add_argument("--out", type=Path,
                        default=Path("results/figures/fail_01_merged_pedestrians.png"))
    return parser.parse_args()


def pedestrian_centers_lidar(frame: dict) -> np.ndarray:
    """KITTI location là tâm đáy box trong camera rectified frame."""
    to_lidar = np.linalg.inv(frame["calib"].T_cam_velo)
    centers = []
    for obj in frame["labels"]:
        if obj.type != "Pedestrian":
            continue
        camera_center = obj.location.copy()
        camera_center[1] -= obj.dimensions[0] / 2
        centers.append((to_lidar @ np.r_[camera_center, 1])[:3])
    return np.asarray(centers)


def matched_clusters(result, centers: np.ndarray) -> list[int]:
    """Ghép GT với duy nhất một AABB chứa tâm GT theo mặt phẳng BEV."""
    matches = []
    for center in centers:
        hits = [index for index, box in enumerate(result.boxes)
                if np.all(center[:2] >= box.min_bound[:2])
                and np.all(center[:2] <= box.max_bound[:2])]
        if len(hits) != 1:
            raise RuntimeError(f"GT center {center[:2]} có {len(hits)} cluster match")
        matches.append(hits[0])
    return matches


def setup_bev(ax, title: str) -> None:
    ax.set_title(title)
    ax.set_xlim(11, 22)
    ax.set_ylim(-4.2, 4.0)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x: phía trước (m)")
    ax.set_ylabel("y: sang trái (m)")
    ax.grid(alpha=0.2)


def mark_gt(ax, centers: np.ndarray) -> None:
    for index, center in enumerate(centers, 1):
        ax.scatter(center[0], center[1], s=125, marker="*", c="#facc15",
                   edgecolors="black", linewidths=0.7, zorder=5)
        ax.annotate(f"P{index}", center[:2], xytext=(4, 6),
                    textcoords="offset points", fontsize=10, weight="bold")


def draw_clusters(ax, result, matches: list[int], centers: np.ndarray) -> None:
    xyz = np.asarray(result.obstacles.points)
    ax.scatter(xyz[:, 0], xyz[:, 1], s=0.7, c="#cbd5e1", linewidths=0)
    for color, cluster_index in zip(("#2563eb", "#ea580c", "#16a34a", "#a855f7"),
                                    sorted(set(matches))):
        cluster_xyz = xyz[result.labels == result.cluster_ids[cluster_index]]
        box = result.boxes[cluster_index]
        ax.scatter(cluster_xyz[:, 0], cluster_xyz[:, 1], s=8, c=color,
                   linewidths=0, zorder=2)
        ax.add_patch(Rectangle(box.min_bound[:2],
                               *(box.max_bound[:2] - box.min_bound[:2]),
                               fill=False, edgecolor=color, linewidth=2, zorder=3))
    mark_gt(ax, centers)


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    args = parse_args()
    paths = frame_paths(args.data_root, args.frame)
    frame = {"points": load_points(args.data_root, args.frame),
             "calib": load_calib(paths["calib"]),
             "labels": load_labels(paths["label"])}
    cropped, _ = crop_kitti_points(frame["points"])
    centers = pedestrian_centers_lidar(frame)
    if len(centers) != 4:
        raise ValueError("Ảnh failure này cần đúng 4 nhãn Pedestrian của KITTI 000043")

    baseline = run_pipeline(cropped, voxel_size=0.20, ground_threshold=0.20,
                            eps=0.60, min_points=8, seed=0)
    smaller_eps = run_pipeline(cropped, voxel_size=0.20, ground_threshold=0.20,
                               eps=0.40, min_points=8, seed=0)
    base_matches = matched_clusters(baseline, centers)
    smaller_matches = matched_clusters(smaller_eps, centers)

    fig = plt.figure(figsize=(13, 10), constrained_layout=True)
    grid = fig.add_gridspec(2, 2, height_ratios=[0.85, 1])
    ax_raw = fig.add_subplot(grid[0, :])
    ax_base = fig.add_subplot(grid[1, 0])
    ax_small = fig.add_subplot(grid[1, 1])
    setup_bev(ax_raw, "Dữ liệu KITTI 000043: bốn tâm GT Pedestrian (P1–P4)")
    ax_raw.scatter(cropped[:, 0], cropped[:, 1], s=1, c="#475569", linewidths=0)
    mark_gt(ax_raw, centers)
    setup_bev(ax_base, f"DBSCAN eps=0,60 m: 4 GT → {len(set(base_matches))} cụm")
    draw_clusters(ax_base, baseline, base_matches, centers)
    setup_bev(ax_small, f"Giảm eps=0,40 m: 4 GT → {len(set(smaller_matches))} cụm")
    draw_clusters(ax_small, smaller_eps, smaller_matches, centers)
    fig.suptitle("Failure CP4: hai người gần nhau bị gộp thành một vật cản\n"
                 "Sao vàng = tâm nhãn GT; khung màu = AABB của cụm DBSCAN")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=170)
    plt.close(fig)
    print(f"GT Pedestrian={len(centers)}; eps=0.60: matches={base_matches}, "
          f"unique_clusters={len(set(base_matches))}; eps=0.40: "
          f"matches={smaller_matches}, unique_clusters={len(set(smaller_matches))}")
    print(f"figure={args.out}")


if __name__ == "__main__":
    main()
