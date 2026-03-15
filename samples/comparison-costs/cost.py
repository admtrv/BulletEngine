import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

WEIGHT = 600
FONT_SIZE = 12
TICK_SIZE = 12
LEGEND_SIZE = 12

plt.rcParams.update({
    "font.weight": WEIGHT,
    "axes.labelweight": WEIGHT,
    "axes.titleweight": WEIGHT,
    "axes.labelsize": FONT_SIZE,
    "xtick.labelsize": TICK_SIZE,
    "ytick.labelsize": TICK_SIZE,
    "legend.fontsize": LEGEND_SIZE,
})

files = {
    "Euler": "data/euler.csv",
    "Midpoint": "data/midpoint.csv",
    "RK4": "data/rk4.csv",
}

order = ["gravity", "+drag", "+coriolis", "+spin"]

pretty = {
    "gravity":   "Gravity",
    "+drag":     "+ Drag",
    "+coriolis": "+ Coriolis",
    "+spin":     "+ Spin Drift",
}

colors = {
    "gravity":   "#F94144",
    "+drag":     "#F9C74F",
    "+coriolis": "#90BE6D",
    "+spin":     "#577590",
}

rows = []
for integ, path in files.items():
    df = pd.read_csv(path)
    means = df.groupby("config")["avg_step_ns"].mean().reindex(order)
    rows.append(means.rename(integ))

M = pd.DataFrame(rows)
M = M.reindex(list(files.keys()))

# grouped bars
x = np.arange(len(M.index))
n_series = len(order)
group_width = 0.82
bar_w = group_width / n_series
offsets = (np.arange(n_series) - (n_series - 1) / 2) * bar_w

plt.figure(figsize=(9.2, 5.6))
ax = plt.gca()

for i, cfg in enumerate(order):
    ax.bar(
        x + offsets[i],
        M[cfg].values,
        width=bar_w * 0.95,
        label=pretty[cfg],
        color=colors[cfg],
        zorder=3,
    )

ax.set_xticks(x)
ax.set_xticklabels(M.index.tolist())
ax.set_ylabel("Average step time [ns]")

for tick in ax.get_xticklabels() + ax.get_yticklabels():
    tick.set_fontweight(WEIGHT)

ax.grid(True, axis="y", alpha=0.35, linewidth=1.0, zorder=0)

leg = ax.legend(
    ncol=5,
    loc="upper center",
    bbox_to_anchor=(0.5, -0.12),
    frameon=True,
)

for t in leg.get_texts():
    t.set_fontweight(WEIGHT)

plt.tight_layout()
plt.show()