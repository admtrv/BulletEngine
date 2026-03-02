import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import FixedLocator, FuncFormatter
import numpy as np

WEIGHT = 600

plt.rcParams.update({
    "font.weight": WEIGHT,
    "axes.labelweight": WEIGHT,
    "axes.titleweight": WEIGHT,
})

df = pd.read_csv("data/convergence.csv")

method_key = {
    "Euler": "euler",
    "Midpoint": "midpoint",
    "RK4": "rk4",
}
df["key"] = df["method"].map(method_key).fillna(df["method"].astype(str).str.lower())

plt.figure(figsize=(8.6, 5.2))
ax = plt.gca()

colors = {
    "euler":    "#f9c74f",
    "midpoint": "#90be6d",
    "rk4":      "#577590",
}

styles = {
    "euler":    dict(linestyle="--", marker="o", markersize=8, linewidth=2.8),
    "midpoint": dict(linestyle="-.", marker="x", markersize=8, linewidth=3.2, mew=2.5),
    "rk4":      dict(linestyle=":",  marker="s", markersize=9, linewidth=3.2),
}

labels = {
    "euler":    "Euler",
    "midpoint": "Midpoint",
    "rk4":      "RK4",
}

draw_order = ["euler", "midpoint", "rk4"]
zorder = {"euler": 1, "midpoint": 2, "rk4": 3}

# plot
for key in draw_order:
    sub = df[df["key"] == key].copy()
    if sub.empty:
        continue
    sub = sub.sort_values("dt")
    ax.plot(
        sub["dt"], sub["error"],
        label=labels[key],
        color=colors[key],
        zorder=zorder[key],
        **styles[key]
    )

ax.set_xscale("log")
ax.set_yscale("log")

ax.set_xlabel("dt, s")
ax.set_ylabel("Error, m")

# X: force ticks like TEST_DT and print them as decimals (no scientific notation)
TEST_DT = [0.04, 0.02, 0.01, 0.005, 0.0025, 0.00125, 0.000625]

def fmt_decimal(x, pos=None):
    return f"{x:.10f}".rstrip("0").rstrip(".")

ax.xaxis.set_major_locator(FixedLocator(TEST_DT))
ax.xaxis.set_major_formatter(FuncFormatter(fmt_decimal))

ax.minorticks_off()

# Y: show as 10^k (plain text), not 1e-k and not mathtext
def fmt_pow10(y, pos=None):
    if y <= 0:
        return ""
    k = int(np.round(np.log10(y)))
    if not np.isclose(y, 10.0**k):
        return ""
    return f"10^{k}"

ymin, ymax = ax.get_ylim()
kmin = int(np.floor(np.log10(ymin)))
kmax = int(np.ceil(np.log10(ymax)))
yticks = [10.0**k for k in range(kmin, kmax + 1)]

ax.yaxis.set_major_locator(FixedLocator(yticks))
ax.yaxis.set_major_formatter(FuncFormatter(fmt_pow10))

for tick in ax.get_xticklabels() + ax.get_yticklabels():
    tick.set_fontweight(WEIGHT)

ax.grid(True, which="major", alpha=0.35, linewidth=1.0)

legend = ax.legend(
    loc="lower center",
    bbox_to_anchor=(0.5, -0.22),
    ncol=3,
    frameon=True
)
for text in legend.get_texts():
    text.set_fontweight(WEIGHT)

plt.tight_layout()
plt.show()