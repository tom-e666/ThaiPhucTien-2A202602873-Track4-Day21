"""Vẽ kết quả quét yaw. Chạy từ gốc repo: python -m src.plot_yaw_sweep"""
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("results/yaw_perturb_sweep.csv", dtype={"frame": str})

fig, ax = plt.subplots(figsize=(7, 4.5))
for frame, g in df.groupby("frame"):
    ax.plot(g["yaw_deg"], 100 * g["hit_ratio"], marker="o", linewidth=2, label=f"Frame {frame}")

ax.set_xlabel("Lệch góc yaw (độ)", fontsize=11)
ax.set_ylabel("% Điểm vật thể rơi đúng trong 2D box", fontsize=11)
ax.set_title("Độ nhạy của LiDAR-Camera Projection theo góc lệch Yaw", fontsize=12, pad=10)
ax.set_ylim(0, 105)
ax.grid(alpha=0.3, linestyle="--")
ax.legend(frameon=True)
fig.tight_layout()

out = Path("results/figures/yaw_sweep.png")
out.parent.mkdir(parents=True, exist_ok=True)
fig.savefig(out, dpi=150)
plt.close(fig)
print(f"-> {out}")

# Vẽ thêm biểu đồ phân tách theo class (Frame 000011 & 000049) nếu có file breakdown
breakdown_path = Path("results/yaw_class_breakdown.csv")
if breakdown_path.exists():
    df_cls = pd.read_csv(breakdown_path, dtype={"frame": str})
    fig2, ax2 = plt.subplots(figsize=(7, 4.5))
    df_f11 = df_cls[df_cls["frame"] == "000011"]
    for c, g in df_f11.groupby("class"):
        ax2.plot(g["yaw_deg"], 100 * g["hit_ratio"], marker="s", linewidth=2, label=f"Frame 000011 - {c}")
    
    ax2.set_xlabel("Lệch góc yaw (độ)", fontsize=11)
    ax2.set_ylabel("% Điểm vật thể rơi đúng trong 2D box", fontsize=11)
    ax2.set_title("Độ suy giảm theo lớp đối tượng (Car vs Pedestrian)", fontsize=12, pad=10)
    ax2.set_ylim(0, 105)
    ax2.grid(alpha=0.3, linestyle="--")
    ax2.legend(frameon=True)
    fig2.tight_layout()
    out2 = Path("results/figures/yaw_class_breakdown.png")
    fig2.savefig(out2, dpi=150)
    plt.close(fig2)
    print(f"-> {out2}")
