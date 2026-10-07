"""Mô-đun công cụ tái sử dụng: LiDAR-Camera Projection QA & Calibration Drift Auditor (Bonus B4).

Công cụ CLI độc lập hỗ trợ QA tự động cho các bài toán Sensor Fusion và Calibration Drift.
Hỗ trợ cả KITTI và nuScenes subset.

Cách dùng:
    python -m src.qa_projection_tool --help
    python -m src.qa_projection_tool
    python -m src.qa_projection_tool --data-root data/kitti_mini --frame 000011 --yaw 1.5 --threshold 0.80
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
import cv2
import numpy as np

from starter.datasets import load_frame
from starter.projection import perturb_extrinsic, project_velo_to_image, velo_to_cam
from src.exp_yaw_sweep import CLASSES, points_in_box


def audit_projection(
    data_root: str = "data/kitti_mini",
    frame: str = "000011",
    yaw: float = 0.0,
    pitch: float = 0.0,
    roll: float = 0.0,
    dx: float = 0.0,
    dy: float = 0.0,
    dz: float = 0.0,
    threshold: float = 0.85,
    save_overlay: str | None = None,
) -> dict:
    """Kiểm tra và đánh giá chất lượng projection kèm calibration drift."""
    fr = load_frame(data_root, frame)
    pts = fr["points"][np.isfinite(fr["points"]).all(axis=1)]

    cam_true = velo_to_cam(pts[:, :3], fr["calib"])
    calib = perturb_extrinsic(
        fr["calib"],
        roll_deg=roll,
        pitch_deg=pitch,
        yaw_deg=yaw,
        t_xyz_m=(dx, dy, dz),
    )

    uv, depth, mask = project_velo_to_image(pts, calib, fr["image"].shape)
    uv_all = np.full((len(pts), 2), np.nan)
    uv_all[mask] = uv

    obj_pts = hits = 0
    class_stats: dict[str, dict] = {}

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

        if obj.type not in class_stats:
            class_stats[obj.type] = {"points": 0, "hits": 0}
        class_stats[obj.type]["points"] += t
        class_stats[obj.type]["hits"] += h

    overall_hit_ratio = round(hits / obj_pts, 4) if obj_pts else 0.0
    passed = overall_hit_ratio >= threshold

    results = {
        "dataset": Path(data_root).name,
        "frame": frame,
        "n_points": len(pts),
        "inside_image": int(mask.sum()),
        "object_points": obj_pts,
        "hit_ratio": overall_hit_ratio,
        "threshold": threshold,
        "passed": passed,
        "class_breakdown": {
            c: {
                "points": s["points"],
                "hit_ratio": round(s["hits"] / s["points"], 4) if s["points"] else 0.0,
            }
            for c, s in class_stats.items()
        },
    }

    if save_overlay:
        out_img = fr["image"].copy()
        for (u, v) in uv.astype(int):
            cv2.circle(out_img, (u, v), 2, (0, 255, 255), -1)
        for obj in fr["labels"]:
            if obj.type in CLASSES:
                x1, y1, x2, y2 = map(int, obj.bbox)
                cv2.rectangle(out_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        out_p = Path(save_overlay)
        out_p.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out_p), out_img)
        results["overlay_path"] = str(out_p)

    return results


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="python -m src.qa_projection_tool",
        description="Mô-đun kiểm định chất lượng LiDAR-Camera Projection QA và phát hiện Calibration Drift.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--data-root", default="data/kitti_mini", help="Thư mục gốc chứa dataset")
    parser.add_argument("--frame", default="000011", help="Mã frame kiểm định")
    parser.add_argument("--yaw", type=float, default=0.0, help="Góc lệch yaw (độ) cần thử nghiệm")
    parser.add_argument("--pitch", type=float, default=0.0, help="Góc lệch pitch (độ) cần thử nghiệm")
    parser.add_argument("--roll", type=float, default=0.0, help="Góc lệch roll (độ) cần thử nghiệm")
    parser.add_argument("--dx", type=float, default=0.0, help="Độ tịnh tiến x (mét)")
    parser.add_argument("--dy", type=float, default=0.0, help="Độ tịnh tiến y (mét)")
    parser.add_argument("--dz", type=float, default=0.0, help="Độ tịnh tiến z (mét)")
    parser.add_argument("--threshold", type=float, default=0.85, help="Ngưỡng hit_ratio tối thiểu để đạt PASS")
    parser.add_argument("--save-overlay", default=None, help="Đường dẫn lưu file ảnh trực quan hoá (nếu cần)")

    args = parser.parse_args()
    res = audit_projection(
        data_root=args.data_root,
        frame=args.frame,
        yaw=args.yaw,
        pitch=args.pitch,
        roll=args.roll,
        dx=args.dx,
        dy=args.dy,
        dz=args.dz,
        threshold=args.threshold,
        save_overlay=args.save_overlay,
    )

    status_tag = "[PASS]" if res["passed"] else "[FAIL - CALIBRATION DRIFT DETECTED]"
    print("=" * 65)
    print(f"PROJECTION QA AUDIT REPORT: {status_tag}")
    print("=" * 65)
    print(f"Dataset: {res['dataset']} | Frame: {res['frame']}")
    print(f"Tổng số điểm LiDAR: {res['n_points']} | Chiếu vào ảnh: {res['inside_image']}")
    print(f"Số điểm trên vật thể: {res['object_points']}")
    print(f"Tỉ lệ điểm hợp lệ (hit_ratio): {res['hit_ratio']*100:.2f}% (Ngưỡng: {res['threshold']*100:.1f}%)")
    print("\nChi tiết theo phân lớp đối tượng:")
    for c, stat in res["class_breakdown"].items():
        print(f"  - {c:<12}: {stat['points']:>5} điểm | hit_ratio: {stat['hit_ratio']*100:.2f}%")

    if "overlay_path" in res:
        print(f"\nẢnh trực quan đã lưu: {res['overlay_path']}")
    print("=" * 65)

    sys.exit(0 if res["passed"] else 1)


if __name__ == "__main__":
    main()
