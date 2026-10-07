"""Tạo hình ảnh phân tích Failure Case cho Topic A: Lệch góc Yaw extrinsic.

Chạy: python -m src.make_failure_case
Lưu ảnh tại: results/figures/fail_01_yaw_drift_pedestrian.png
"""
from pathlib import Path
import cv2
import numpy as np

from starter.datasets import load_frame
from starter.projection import perturb_extrinsic, project_velo_to_image, velo_to_cam
from src.exp_yaw_sweep import points_in_box


def render_projected(fr: dict, yaw_deg: float) -> tuple[np.ndarray, np.ndarray, float]:
    img = fr["image"].copy()
    pts = fr["points"][np.isfinite(fr["points"]).all(axis=1)]
    cam_true = velo_to_cam(pts[:, :3], fr["calib"])
    calib = perturb_extrinsic(fr["calib"], yaw_deg=yaw_deg) if yaw_deg != 0 else fr["calib"]

    uv, depth, mask = project_velo_to_image(pts, calib, img.shape)
    uv_all = np.full((len(pts), 2), np.nan)
    uv_all[mask] = uv

    # Vẽ điểm lidar màu sắc theo depth
    norm_depth = np.clip((depth - 2.0) / 40.0, 0, 1)
    for (u, v), d in zip(uv.astype(int), norm_depth):
        color = (int(255 * (1 - d)), int(255 * d), 50)  # BGR
        cv2.circle(img, (u, v), 2, color, -1)

    ped_hits = 0
    ped_pts = 0
    # Vẽ 2D box của nhãn
    for obj in fr["labels"]:
        x1, y1, x2, y2 = map(int, obj.bbox)
        if obj.type == "Pedestrian":
            color = (0, 255, 0) if yaw_deg == 0 else (0, 0, 255)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            sel = points_in_box(cam_true, obj) & mask
            u_obj = uv_all[sel, 0]
            v_obj = uv_all[sel, 1]
            hits = int(((u_obj >= x1) & (u_obj <= x2) & (v_obj >= y1) & (v_obj <= y2)).sum())
            ped_hits += hits
            ped_pts += int(sel.sum())
        elif obj.type == "Car":
            cv2.rectangle(img, (x1, y1), (x2, y2), (255, 200, 0), 1)

    hit_ratio = (ped_hits / ped_pts) if ped_pts else 1.0
    return img, uv, hit_ratio


def main() -> None:
    fr = load_frame("data/kitti_mini", "000011")

    img_ok, _, hr_ok = render_projected(fr, yaw_deg=0.0)
    img_fail, _, hr_fail = render_projected(fr, yaw_deg=2.0)

    # Thêm text tiêu đề trên ảnh lớn
    cv2.putText(img_ok, f"Baseline (Yaw 0.0 deg) - Ped Hit: {hr_ok*100:.1f}%", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(img_fail, f"Failure (Yaw +2.0 deg) - Ped Hit: {hr_fail*100:.1f}%", (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA)

    # Crop vùng người đi bộ ở 34m (obj #3: bbox [649, 168, 665, 206])
    # Mở rộng vùng crop để thấy độ lệch điểm: [130:240, 620:700]
    crop_ok = img_ok[135:235, 615:705]
    crop_fail = img_fail[135:235, 615:705]

    # Phóng to vùng crop 3x
    zoom_ok = cv2.resize(crop_ok, (270, 300), interpolation=cv2.INTER_NEAREST)
    zoom_fail = cv2.resize(crop_fail, (270, 300), interpolation=cv2.INTER_NEAREST)

    cv2.putText(zoom_ok, "Zoom: Pedestrian at 34m", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(zoom_ok, "Points inside box: OK", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1, cv2.LINE_AA)

    cv2.putText(zoom_fail, "Zoom: Pedestrian at 34m", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(zoom_fail, "Points shifted OUT (~25px)", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1, cv2.LINE_AA)

    # Vẽ mũi tên chỉ thị trôi dạt điểm trên zoom_fail
    cv2.arrowedLine(zoom_fail, (110, 150), (180, 150), (0, 0, 255), 2, tipLength=0.3)

    # Ghép ảnh side-by-side
    top_bar = np.hstack([img_ok, img_fail])
    # Tạo banner phía dưới chứa 2 ảnh zoom
    # Chiều rộng mỗi nửa là img_ok.shape[1] = 1242 (KITTI)
    h_top, w_top = top_bar.shape[:2]
    bottom_bar = np.zeros((320, w_top, 3), dtype=np.uint8)
    
    # Canh giữa 2 ảnh zoom ở mỗi nửa trái và phải
    offset_left = (w_top // 4) - (zoom_ok.shape[1] // 2)
    offset_right = (3 * w_top // 4) - (zoom_fail.shape[1] // 2)
    bottom_bar[10:10+zoom_ok.shape[0], offset_left:offset_left+zoom_ok.shape[1]] = zoom_ok
    bottom_bar[10:10+zoom_fail.shape[0], offset_right:offset_right+zoom_fail.shape[1]] = zoom_fail

    final_img = np.vstack([top_bar, bottom_bar])

    out_path = Path("results/figures/fail_01_yaw_drift_pedestrian.png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), final_img)
    print(f"-> Saved failure case figure: {out_path}")


if __name__ == "__main__":
    main()
