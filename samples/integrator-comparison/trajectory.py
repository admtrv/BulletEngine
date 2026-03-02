import pandas as pd
import matplotlib.pyplot as plt

WEIGHT = 600

plt.rcParams.update({
    "font.weight": WEIGHT,
    "axes.labelweight": WEIGHT,
    "axes.titleweight": WEIGHT,
})

df = pd.read_csv("data/trajectory.csv")

plt.figure(figsize=(8.6, 5.2))

# https://coolors.co/palette/f94144-f3722c-f8961e-f9c74f-90be6d-43aa8b-577590
colors = {
    "euler":      "#f9c74f",
    "midpoint":   "#90be6d",
    "rk4":        "#577590",
    "analytical": "#f94144",
}

styles = {
    "euler":      dict(linestyle="--", marker="o", markersize=9,  linewidth=2.8),
    "midpoint":   dict(linestyle="-.", marker="x", markersize=8,  linewidth=3.2, mew=2.5),
    "rk4":        dict(linestyle=":",  marker="s", markersize=10, linewidth=3.2),
    "analytical": dict(linestyle="-",  marker="+", markersize=12, linewidth=3.2, mew=2.5),
}

labels = {
    "euler":      "Euler",
    "midpoint":   "Midpoint",
    "rk4":        "RK4",
    "analytical": "Exact Solution",
}

draw_order = ["euler", "midpoint", "analytical", "rk4"]
zorder = {"euler": 1, "midpoint": 2, "analytical": 3, "rk4": 4}

for name in draw_order:
    plt.plot(
        df[f"{name}_x"], df[f"{name}_y"],
        label=labels[name],
        color=colors[name],
        zorder=zorder[name],
        **styles[name]
    )

plt.xlabel("x, m")
plt.ylabel("y, m")

ax = plt.gca()

for tick in ax.get_xticklabels() + ax.get_yticklabels():
    tick.set_fontweight(WEIGHT)

plt.grid(True, alpha=0.35, linewidth=1.0)

desired_labels = ["Exact Solution", "Euler", "Midpoint", "RK4"]
handles, current_labels = ax.get_legend_handles_labels()
idx = [current_labels.index(lbl) for lbl in desired_labels]
handles = [handles[i] for i in idx]
ordered_labels = [current_labels[i] for i in idx]

legend = plt.legend(
    handles, ordered_labels,
    loc="lower center",
    bbox_to_anchor=(0.5, -0.22),
    ncol=4,
    frameon=True
)

for text in legend.get_texts():
    text.set_fontweight(WEIGHT)

plt.tight_layout()
plt.show()