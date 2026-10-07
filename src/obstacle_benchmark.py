"""CP3: quét voxel_size và ngưỡng RANSAC độc lập trên ba frame KITTI."""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
import sys
import tempfile
from time import perf_counter

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "topic_d_matplotlib"))
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from starter.datasets import dataset_type, load_points
from src.obstacle_core import PipelineResult, crop_kitti_points, run_pipeline


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", default="data/kitti_mini")
    parser.add_argument("--frames", nargs="+", default=["000011", "000043", "000049"])
    parser.add_argument("--voxel-levels", nargs="+", type=float, default=[0.10, 0.20, 0.40])
    parser.add_argument("--ground-levels", nargs="+", type=float, default=[0.10, 0.20, 0.30])
    parser.add_argument("--base-voxel", type=float, default=0.20)
    parser.add_argument("--base-ground", type=float, default=0.20)
    parser.add_argument("--eps", type=float, default=0.60)
    parser.add_argument("--min-points", type=int, default=8)
    parser.add_argument("--max-forward", type=float, default=30.0)
    parser.add_argument("--half-width", type=float, default=15.0)
    parser.add_argument("--near-range", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--repeats", type=int, default=20,
                        help="Số lần đo sau một lượt khởi động cho mỗi cấu hình")
    parser.add_argument("--out", type=Path, default=Path("results/obstacle_parameter_sweep.csv"))
    parser.add_argument("--latency-out", type=Path, default=Path("results/obstacle_latency.csv"))
    parser.add_argument("--figure-out", type=Path,
                        default=Path("results/figures/obstacle_parameter_sweep.png"))
    args = parser.parse_args()
    if not args.frames or len(args.voxel_levels) < 3 or len(args.ground_levels) < 3:
        parser.error("Cần ít nhất 1 frame và ít nhất 3 mức cho mỗi tham số")
    if any(v <= 0 for v in (*args.voxel_levels, *args.ground_levels,
                            args.base_voxel, args.base_ground, args.eps,
                            args.max_forward, args.half_width, args.near_range)):
        parser.error("Các tham số khoảng cách phải > 0")
    if args.min_points < 1 or args.repeats < 20:
        parser.error("min-points >= 1 và repeats >= 20 theo yêu cầu CP3")
    if args.base_voxel not in args.voxel_levels or args.base_ground not in args.ground_levels:
        parser.error("Mức baseline phải nằm trong danh sách mức quét tương ứng")
    return args


def nearest_xy_distance(box) -> float:
    """Khoảng cách từ gốc LiDAR tới hình chữ nhật BEV của AABB."""
    xy_min, xy_max = box.min_bound[:2], box.max_bound[:2]
    gap = np.maximum(np.maximum(xy_min, -xy_max), 0.0)
    return float(np.linalg.norm(gap))


def geometry_metrics(result: PipelineResult, near_range: float) -> dict:
    distances = np.array([nearest_xy_distance(box) for box in result.boxes])
    sizes = np.array([box.get_extent() for box in result.boxes]).reshape(-1, 3)
    return {
        "n_voxel_points": len(result.sampled.points),
        "n_ground_points": len(result.ground.points),
        "n_non_ground_points": len(result.obstacles.points),
        "n_clusters": len(result.boxes),
        "n_near_clusters": int(np.count_nonzero(distances <= near_range)),
        "nearest_cluster_m": round(float(distances.min()), 4) if len(distances) else "",
        "median_box_length_m": round(float(np.median(sizes[:, 0])), 4) if len(sizes) else "",
        "median_box_width_m": round(float(np.median(sizes[:, 1])), 4) if len(sizes) else "",
        "median_box_height_m": round(float(np.median(sizes[:, 2])), 4) if len(sizes) else "",
        "plane_abs_nz": round(abs(float(result.plane[2])), 4),
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def plot_results(rows: list[dict], args: argparse.Namespace) -> None:
    fig, axes = plt.subplots(3, 2, figsize=(12, 11), constrained_layout=True)
    sweeps = (("voxel_size_m", "ground_threshold_m", args.base_ground,
               "Voxel size (m)"),
              ("ground_threshold_m", "voxel_size_m", args.base_voxel,
               "Ngưỡng tách đất (m)"))
    metrics = (("n_near_clusters", f"Số cụm gần (≤{args.near_range:g} m)"),
               ("nearest_cluster_m", "Khoảng cách cụm gần nhất (m)"),
               ("latency_p50_ms", "Latency p50 (ms)"))
    for col, (variable, fixed, fixed_value, x_label) in enumerate(sweeps):
        for frame in args.frames:
            selection = sorted((r for r in rows if r["frame_id"] == frame
                                and r[fixed] == fixed_value), key=lambda r: r[variable])
            x = [r[variable] for r in selection]
            for row_index, (metric, title) in enumerate(metrics):
                ax = axes[row_index, col]
                ax.plot(x, [r[metric] for r in selection], marker="o", label=frame)
                ax.set_title(title)
                ax.set_xlabel(x_label)
                ax.grid(alpha=0.25)
                if row_index == 0:
                    ax.legend(title="Frame", fontsize=8)
    fig.suptitle("Topic D | quét từng tham số, giữ các yếu tố khác cố định")
    args.figure_out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.figure_out, dpi=160)
    plt.close(fig)


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8")
    args = parse_args()
    if dataset_type(args.data_root) != "kitti":
        raise ValueError("Benchmark này dùng hệ trục KITTI")

    configs = [(v, args.base_ground) for v in args.voxel_levels]
    configs += [(args.base_voxel, g) for g in args.ground_levels if g != args.base_ground]
    rows, latency_rows = [], []
    for frame in args.frames:
        raw = load_points(args.data_root, frame)
        cropped, invalid = crop_kitti_points(raw, args.max_forward, args.half_width)
        for voxel, ground in configs:
            settings = (cropped, voxel, ground, args.eps, args.min_points, args.seed)
            warmup = run_pipeline(*settings)  # Không tính lần khởi động vào latency.
            metrics = geometry_metrics(warmup, args.near_range)
            times_ms = []
            for repeat in range(args.repeats):
                start = perf_counter()
                result = run_pipeline(*settings)
                elapsed_ms = (perf_counter() - start) * 1000
                if geometry_metrics(result, args.near_range) != metrics:
                    raise RuntimeError(f"Kết quả hình học không ổn định: {frame}, {voxel}, {ground}")
                times_ms.append(elapsed_ms)
                latency_rows.append({"frame_id": frame, "voxel_size_m": voxel,
                                     "ground_threshold_m": ground, "repeat": repeat + 1,
                                     "latency_ms": round(elapsed_ms, 4)})
            row = {"frame_id": frame, "data_root": args.data_root,
                   "voxel_size_m": voxel, "ground_threshold_m": ground,
                   "eps_m": args.eps, "min_points": args.min_points,
                   "max_forward_m": args.max_forward, "half_width_m": args.half_width,
                   "near_range_m": args.near_range, "seed": args.seed,
                   "ransac_iterations": 100, "n_repeats": args.repeats,
                   "n_raw_points": len(raw), "n_invalid_points": invalid,
                   "n_roi_points": len(cropped), **metrics,
                   "latency_p50_ms": round(float(np.percentile(times_ms, 50)), 3),
                   "latency_p95_ms": round(float(np.percentile(times_ms, 95)), 3)}
            rows.append(row)
            print(f"{frame} voxel={voxel:.2f} ground={ground:.2f}: "
                  f"clusters={metrics['n_clusters']} near={metrics['n_near_clusters']} "
                  f"nearest={metrics['nearest_cluster_m']}m "
                  f"p50={row['latency_p50_ms']}ms")

    write_csv(args.out, rows)
    write_csv(args.latency_out, latency_rows)
    plot_results(rows, args)
    print(f"saved: {args.out}, {args.latency_out}, {args.figure_out}")


if __name__ == "__main__":
    main()
