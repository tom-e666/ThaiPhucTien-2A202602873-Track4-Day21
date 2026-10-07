"""Bonus B3: Đo latency của projection pipeline theo chuẩn benchmark (21 lần chạy, bỏ lần 1).

Chạy: python -m src.bench_latency
"""
import csv
from pathlib import Path
import time
import numpy as np

from starter.datasets import load_frame
from starter.projection import project_velo_to_image


def run_pipeline(fr: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    pts = fr["points"][np.isfinite(fr["points"]).all(axis=1)]
    return project_velo_to_image(pts, fr["calib"], fr["image"].shape)


def main() -> None:
    fr = load_frame("data/kitti_mini", "000011")
    n_iters = 21

    records = []
    times_sec = []

    for i in range(n_iters):
        t0 = time.perf_counter()
        _ = run_pipeline(fr)
        dt = time.perf_counter() - t0
        times_sec.append(dt)
        records.append({
            "iteration": i,
            "is_warmup": (i == 0),
            "latency_ms": round(dt * 1000, 3)
        })

    valid_times_ms = np.array(times_sec[1:]) * 1000
    p50 = np.percentile(valid_times_ms, 50)
    p95 = np.percentile(valid_times_ms, 95)
    mean = np.mean(valid_times_ms)

    out_csv = Path("results/latency_benchmark.csv")
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["iteration", "is_warmup", "latency_ms"])
        writer.writeheader()
        writer.writerows(records)

    print(f"-> Đã ghi: {out_csv} ({len(records)} dòng)")
    print(f"Hardware: 11th Gen Intel(R) Core(TM) i5-11400H @ 2.70GHz, 24 GB RAM, NVIDIA RTX 3050 Ti")
    print(f"Warmup latency: {records[0]['latency_ms']:.2f} ms")
    print(f"Latency benchmark (20 runs excluding warmup):")
    print(f"  p50  = {p50:.2f} ms")
    print(f"  p95  = {p95:.2f} ms")
    print(f"  mean = {mean:.2f} ms")


if __name__ == "__main__":
    main()
