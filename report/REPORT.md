# Báo cáo Day 6: LiDAR-Camera Projection QA & Calibration Drift

> Thay **mọi** ô có chữ ĐIỀN nằm trong ngoặc vuông bằng nội dung của bạn, xoá luôn cả dấu ngoặc vuông. Lệnh `python tools/check_submission.py` sẽ báo FAIL nếu còn sót bất kỳ chỗ nào.

- **Họ tên:** Thái Phúc Tiến
- **MSSV:** 2A202602873
- **Lớp:** AI20K-T4
- **Link repo:** https://github.com/tom-e666/ThaiPhucTien-2A202602873-Track4-Day21
- **Topic:** A — LiDAR-camera projection QA
- **Dataset:** data/kitti_mini, data/synthetic
- **Các frame đã dùng:** 000008, 000011, 000049

> Hãy viết ngắn: mỗi mục từ 3 đến 8 dòng, ưu tiên số liệu và hình ảnh.

## 1. Claim

Một câu khẳng định kỹ thuật có thể kiểm chứng. Ví dụ: *"Lệch yaw 1° làm 12% điểm LiDAR rơi ra khỏi vật thể ở 30 m, phát hiện được bằng edge-alignment score với ngưỡng X."*

Lệch góc yaw extrinsic 1° làm tỉ lệ điểm LiDAR rơi đúng vào 2D bounding box của người đi bộ giảm hơn 20% (từ 99.5% xuống 77.4%), trong khi ít ảnh hưởng hơn tới xe ô tô cỡ lớn (giảm dưới 1.5%), do vật thể hẹp ở cự ly xa có kích thước pixel nhỏ trên ảnh nên cực kỳ nhạy cảm với sai số góc quay.

## 2. Evidence

Bảng hoặc plot số liệu, kèm ảnh/video demo. Dữ liệu từ file `results/yaw_perturb_sweep.csv` và `results/yaw_class_breakdown.csv`.

| Cấu hình (yaw) | Frame 000008 (Xe hơi) | Frame 000011 (Người đi bộ) | Frame 000049 (Hỗn hợp) | Ghi chú |
|---|---|---|---|---|
| 0.0° (Gốc) | 99.63% | 99.45% | 99.25% | Mức sàn calib chuẩn |
| 0.5° | 99.57% | 91.88% | 97.46% | Bắt đầu trôi điểm ở pedestrian |
| 1.0° | 98.62% | 77.44% | 93.50% | Người đi bộ giảm mạnh > 20% |
| 2.0° | 94.81% | 45.44% | 84.74% | Sai lệch nghiêm trọng |
| 3.0° | 90.98% | 21.23% | 74.32% | Pedestrian trượt gần hết (>94%) |

![yaw sweep](../results/figures/yaw_sweep.png)
![yaw class breakdown](../results/figures/yaw_class_breakdown.png)

**Nhận xét xu hướng:**
- Lệch góc yaw ảnh hưởng mạnh nhất tới frame `000011` (nhiều người đi bộ): ở 1.0°, tỉ lệ hit_ratio giảm từ 99.45% xuống 77.44%. Khi bóc tách chi tiết lớp đối tượng trong frame `000011`, hit_ratio của `Pedestrian` giảm sâu xuống còn 61.89%, trong khi `Car` vẫn duy trì 91.12%.
- Frame `000008` (chủ yếu là xe ô tô kích thước lớn) suy giảm rất chậm: ở 1.0° đạt 98.62% và ở 3.0° vẫn giữ 90.98%, vì kích thước bounding box của xe hơi lớn hơn người đi bộ gấp nhiều lần trên ảnh.
- Ở mức lệch 3.0°, điểm LiDAR của người đi bộ chỉ còn 5.21% rơi đúng vào box, gần như làm tê liệt khả năng sensor fusion đối với người đi bộ.

## 3. Failure case

Nêu khi nào hệ thống hoặc phương pháp fail, vì sao fail, và liên hệ tới lớp nào trong 6 lớp debug: I/O, Geometry, Time, Preprocess, Model, Metric.

![failure](../results/figures/fail_[ĐIỀN].png)

[ĐIỀN]

## 4. Khuyến nghị nếu triển khai thật

Use-case cụ thể (ADAS / robot / drone), trade-off và bước tiếp theo.

[ĐIỀN]

## 5. Cách chạy lại

Các lệnh tái tạo lại toàn bộ kết quả từ repo sạch:

```bash
# 1. Chạy self-test projection logic
python -m src.test_projection

# 2. Render ảnh chiếu overlay mẫu
python -m starter.projection --data-root data/kitti_mini --frame 000011

# 3. Chạy thí nghiệm quét góc lệch yaw (tạo kết quả CSV và breakdown)
python -m src.exp_yaw_sweep --data-root data/kitti_mini --frames 000008 000011 000049

# 4. Vẽ đồ thị phân tích
python -m src.plot_yaw_sweep
```

## 6. Khai báo sử dụng AI

Dùng script mẫu của codelab làm điểm xuất phát, sau đó mở rộng thêm hàm phân tách đa lớp (`run_breakdown`) để bóc tách độ nhạy hit_ratio riêng biệt giữa Car và Pedestrian.

| Công cụ | Dùng cho việc gì | Bạn đã kiểm chứng thế nào |
|---|---|---|
| Google Antigravity | Gợi ý cấu trúc sweep, script vẽ đồ thị và hàm mở rộng `run_breakdown` phân tích per-class | Tự chạy self-test `test_projection`, đối chiếu bảng kết quả kỳ vọng và kiểm tra tái lập 100% bằng script `filecmp` |

