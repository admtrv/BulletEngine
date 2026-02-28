import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

WEIGHT = 600

plt.rcParams.update({
    "font.weight": WEIGHT,
    "axes.labelweight": WEIGHT,
    "axes.titleweight": WEIGHT,
})

df = pd.read_csv("data.csv")

colors = {
    "euler":      "#f9c74f",
    "midpoint":   "#90be6d",
    "rk4":        "#577590",
}

step_cols = {
    "Euler":    "euler_step_ns",
    "Midpoint": "midpoint_step_ns",
    "RK4":      "rk4_step_ns",
}

means_ns = {label: df[col].dropna().mean() for label, col in step_cols.items()}

plt.figure(figsize=(8.6, 5.2))
ax = plt.gca()

bars = ax.bar(
    list(means_ns.keys()),
    list(means_ns.values()),
    color=[colors["euler"], colors["midpoint"], colors["rk4"]],
    zorder=3,
)

ax.set_ylabel("Average step time, ns")

for tick in ax.get_xticklabels() + ax.get_yticklabels():
    tick.set_fontweight(WEIGHT)

ax.grid(True, axis="y", alpha=0.35, linewidth=1.0, zorder=0)

for b in bars:
    h = b.get_height()
    ax.text(
        b.get_x() + b.get_width() / 2,
        h,
        f"{h:.0f}",
        ha="center",
        va="bottom",
        fontweight=WEIGHT,
    )

plt.tight_layout()
plt.show()