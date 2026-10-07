"""Bonus B2: Stress test suy giảm dữ liệu (Perturbation Robustness).

Thử nghiệm:
1. random_dropout: keep_ratio = 1.0, 0.7, 0.5, 0.3
2. gaussian_noise: sigma_xyz_m = 0.0, 0.02, 0.05, 0.10

Chạy: python -m src.stress_test_perturb
"""
import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from starter.datasets import load_frame
from starter.perturb import gaussian_noise, random_dropout
from starter.projection import project_velo_to_image, velo_to_cam
from src.exp_yaw_sweep import CLASSES, points_in_box


def eval_points(fr: dict, pts: np.ndarray) -> dict:
    pts_valid = pts[np.isfinite(pts).all(axis=1)]
    cam_true = velo_to_cam(pts_valid[:, :3], fr["calib"])
    uv, _, mask = project_velo_to_image(pts_valid, fr["calib"], fr["image"].shape)
    uv_all = np.full((len(pts_valid), 2), np.nan)
    uv_all[mask] = uv

    obj_pts = hits = 0
    ped_pts = ped_hits = 0

    for obj in fr["labels"]:
        if obj.type not in CLASSES:
            continue
        sel = points_in_box(cam_true, obj) & mask
        u, v = uv_all[sel, 0], uv_all[sel, 1]
        x1, y1, x2, y2 = obj.bbox
        h = int(((u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)).sum())
        t = int(sel.sum())
        hits += h
        obj_pts += t
        if obj.type == "Pedestrian":
            ped_hits += h
            ped_pts += t

    return {
        "n_points": len(pts_valid),
        "inside_image": int(mask.sum()),
        "object_points": obj_pts,
        "hit_ratio": round(hits / obj_pts, 4) if obj_pts else 0.0,
        "ped_points": ped_pts,
        "ped_hit_ratio": round(ped_hits / ped_pts, 4) if ped_pts else 0.0,
    }


def main() -> None:
    fr = load_frame("data/kitti_mini", "000011")
    rows = []

    # 1. Random Dropout sweep
    for keep in [1.0, 0.7, 0.5, 0.3]:
        p = random_dropout(fr["points"], keep_ratio=keep, seed=0)
        res = eval_points(fr, p)
        rows.append({
            "perturb_type": "random_dropout",
            "param_value": keep,
            "param_unit": "keep_ratio",
            **res
        })

    # 2. Gaussian Noise sweep
    for sigma in [0.0, 0.02, 0.05, 0.10]:
        p = gaussian_noise(fr["points"], sigma_xyz_m=sigma, seed=0)
        res = eval_points(fr, p)
        rows.append({
            "perturb_type": "gaussian_noise",
            "param_value": sigma,
            "param_unit": "sigma_xyz_m",
            **res
        })

    out_csv = Path("results/stress_test_perturb.csv")
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"-> {out_csv} ({len(rows)} dòng)")

    # Vẽ đồ thị 2 subplot
    df = pd.DataFrame(rows)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.5))

    df_drop = df[df["perturb_type"] == "random_dropout"]
    ax1.plot(df_drop["param_value"], df_drop["object_points"], marker="o", color="blue", label="Tổng điểm trên vật thể")
    ax1.plot(df_drop["param_value"], df_drop["ped_points"], marker="s", color="orange", label="Điểm trên người đi bộ")
    ax1.set_xlabel("Tỉ lệ giữ lại điểm (keep_ratio)")
    ax1.set_ylabel("Số lượng điểm LiDAR")
    ax1.set_title("Stress Test: Mất điểm ngẫu nhiên (Dropout)")
    ax1.grid(alpha=0.3, linestyle="--")
    ax1.legend()

    df_noise = df[df["perturb_type"] == "gaussian_noise"]
    ax2.plot(df_noise["param_value"] * 100, df_noise["hit_ratio"] * 100, marker="o", color="green", label="Tổng hit_ratio (%)")
    ax2.plot(df_noise["param_value"] * 100, df_noise["ped_hit_ratio"] * 100, marker="^", color="red", label="Pedestrian hit_ratio (%)")
    ax2.set_xlabel("Độ lệch chuẩn nhiễu Sigma (cm)")
    ax2.set_ylabel("Tỉ lệ điểm trong box (%)")
    ax2.set_title("Stress Test: Nhiễu vị trí không gian (Gaussian)")
    ax2.set_ylim(80, 102)
    ax2.grid(alpha=0.3, linestyle="--")
    ax2.legend()

    fig.tight_layout()
    out_fig = Path("results/figures/stress_test_perturb.png")
    fig.savefig(out_fig, dpi=150)
    plt.close(fig)
    print(f"-> {out_fig}")


if __name__ == "__main__":
    main()
