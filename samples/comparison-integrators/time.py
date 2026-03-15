import pandas as pd
import matplotlib.pyplot as plt

WEIGHT = 600
FONT_SIZE = 11
TICK_SIZE = 11
VALUE_SIZE = 11

plt.rcParams.update({
    "font.weight": WEIGHT,
    "axes.labelweight": WEIGHT,
    "axes.titleweight": WEIGHT,
    "axes.labelsize": FONT_SIZE,
    "xtick.labelsize": TICK_SIZE,
    "ytick.labelsize": TICK_SIZE,
})

df = pd.read_csv("data/time.csv")

colors = {
    "euler":      "#f9c74f",
    "midpoint":   "#90be6d",
    "rk4":        "#577590",
}

order = ["euler", "midpoint", "rk4"]

pretty = {
    "euler":    "Euler",
    "midpoint": "Midpoint",
    "rk4":      "RK4",
}

means_ns = (
    df.groupby("integrator", as_index=True)["avg_step_ns"]
      .mean()
      .reindex(order)
)

plt.figure(figsize=(8.6, 5.2))
ax = plt.gca()

bars = ax.bar(
    [pretty[i] for i in means_ns.index],
    means_ns.values.tolist(),
    color=[colors[i] for i in means_ns.index],
    zorder=3,
)

ax.set_ylabel("Average step time [ns]")

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
        fontsize=VALUE_SIZE,
    )

plt.tight_layout()
plt.show()