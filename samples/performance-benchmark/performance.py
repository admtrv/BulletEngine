import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import inset_axes, mark_inset

WEIGHT = 600

plt.rcParams.update({
    "font.weight": WEIGHT,
    "axes.labelweight": WEIGHT,
    "axes.titleweight": WEIGHT,
})

files = {
    "1":    "data/1.csv",
    "10":   "data/10.csv",
    "100":  "data/100.csv",
    "1000": "data/1000.csv",
}

colors = {
    "1":    "#F94144",
    "10":   "#f9c74f",
    "100":  "#90be6d",
    "1000": "#577590",
}

draw_order = ["1000", "100", "10", "1"]
zorder = {"1000": 1, "100": 2, "10": 3, "1": 4}

MED_WINDOW = 5
MEAN_WINDOW = 15

plt.figure(figsize=(8.6, 5.2))
ax = plt.gca()

series = {}

for label in draw_order:
    df = pd.read_csv(files[label]).sort_values("step")
    df["step_ms"] = df["step_ns"] / 1_000_000.0

    mean_ms = df["step_ms"].mean()
    median_ms = df["step_ms"].median()
    print(f"{label:>4} projectiles | mean {mean_ms:.4f} ms | median {median_ms:.4f} ms")

    y = df["step_ms"]
    y = y.rolling(window=MED_WINDOW, center=False, min_periods=1).median()
    y = y.rolling(window=MEAN_WINDOW, center=False, min_periods=1).mean()

    series[label] = (df["step"], y)

    ax.plot(
        df["step"], y,
        label=label,
        color=colors[label],
        linewidth=3.2,
        zorder=zorder[label],
    )

ax.set_xlabel("Step")
ax.set_ylabel("Step time, ms")

ax.ticklabel_format(axis="y", style="plain", useOffset=False)

for tick in ax.get_xticklabels() + ax.get_yticklabels():
    tick.set_fontweight(WEIGHT)

ax.grid(True, axis="y", alpha=0.35, linewidth=1.0, zorder=0)

legend = ax.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, -0.22),
    ncol=4,
    frameon=True
)
for text in legend.get_texts():
    text.set_fontweight(WEIGHT)

# zoom-in inset
axins = inset_axes(ax, width="36%", height="36%", loc="upper right", borderpad=1.2)

for label in draw_order:
    x, y = series[label]
    axins.plot(
        x, y,
        color=colors[label],
        linewidth=2.2,
        zorder=zorder[label],
    )

x1, x2 = 1500, 2000
y1, y2 = 0.000, 0.100

axins.set_xlim(x1, x2)
axins.set_ylim(y1, y2)

axins.ticklabel_format(axis="y", style="plain", useOffset=False)
axins.grid(True, axis="y", alpha=0.25, linewidth=0.8)

for tick in axins.get_xticklabels() + axins.get_yticklabels():
    tick.set_fontweight(WEIGHT)
    tick.set_fontsize(9)

mark_inset(ax, axins, loc1=2, loc2=4, fc="none", ec="0.45", lw=1.0)

plt.tight_layout()
plt.show()