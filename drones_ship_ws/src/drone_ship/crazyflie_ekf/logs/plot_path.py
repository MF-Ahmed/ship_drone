#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.ticker import MultipleLocator  # for tick spacing

# ------------- CONFIG -------------
CSV_PATH = "drone_odom_20251215_112916.csv"   # <-- change if needed

DRONE_WHITELIST = {"drone1", "drone2", "drone3"}
VALID_SOURCES   = {"raw", "ekf"}              # add "gt" if you also log GT

# Filtering
MAX_ABS_POS = 500.0   # keep only reasonable positions
MAX_STEP    = 5.0     # max XY jump between samples [m]
MAX_SPEED   = 6.0     # max speed between samples [m/s]
PLOT_EVERY  = 2       # time/point downsampling (1 = no downsample)

# TICK STEP (spacing between ticks, NOT font size)
X_TICK_STEP = 10.0    # [m]
Y_TICK_STEP = 10.0    # [m]

# Ignore early noisy EKF samples (seconds from first EKF sample)
WARMUP_S = 2.0

# Aesthetics for a single-column IROS figure
plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 5,
    "axes.labelsize": 5,
    "axes.titlesize": 7,
    "legend.fontsize": 5,
    "xtick.labelsize": 5,
    "ytick.labelsize": 5,
})

# Per-drone colors (for markers)
DRONE_COLORS = {
    "drone1": "#1f77b4",  # blue
    "drone2": "#ff7f0e",  # orange
    "drone3": "#2ca02c",  # green
}

RAW_COLOR = "#888888"    # grey for raw odom


# -------- LOAD & BASIC CLEAN --------
print(f"Loading {CSV_PATH} ...")
df = pd.read_csv(CSV_PATH, engine="python", on_bad_lines="skip")

required = {"stamp", "drone", "source", "x", "y"}
missing = required - set(df.columns)
if missing:
    raise RuntimeError(f"CSV missing columns: {missing}")

df["drone"]  = df["drone"].astype(str).str.strip()
df["source"] = df["source"].astype(str).str.strip().str.lower()

df["x"]     = pd.to_numeric(df["x"], errors="coerce")
df["y"]     = pd.to_numeric(df["y"], errors="coerce")
df["stamp"] = pd.to_numeric(df["stamp"], errors="coerce")

df = df.dropna(subset=["x", "y", "stamp"])
df = df[df["drone"].isin(DRONE_WHITELIST)]
df = df[df["source"].isin(VALID_SOURCES)]

# Pos bounds
df = df[(df["x"].abs() <= MAX_ABS_POS) & (df["y"].abs() <= MAX_ABS_POS)]

if df.empty:
    raise RuntimeError("No usable rows left after cleaning.")

print("Drones in file:", sorted(df["drone"].unique()))
print("Sources:", sorted(df["source"].unique()))


def filter_track(track: pd.DataFrame) -> pd.DataFrame:
    """Remove large jumps / unrealistic speeds for a single (drone, source) track."""
    track = track.sort_values("stamp").copy()
    if len(track) < 3:
        return track

    dx = track["x"].diff().to_numpy()
    dy = track["y"].diff().to_numpy()
    dt = track["stamp"].diff().to_numpy()

    step = np.sqrt(dx**2 + dy**2)
    with np.errstate(divide="ignore", invalid="ignore"):
        speed = step / dt

    mask = np.ones(len(track), dtype=bool)
    bad = (step > MAX_STEP) | (speed > MAX_SPEED) | np.isnan(speed)
    mask[bad] = False
    mask[0] = True  # always keep first sample

    return track[mask]


# Filter per drone+source
tracks_filtered = []
for drone in sorted(df["drone"].unique()):
    df_d = df[df["drone"] == drone]
    for src in VALID_SOURCES:
        df_s = df_d[df_d["source"] == src]
        if df_s.empty:
            continue
        tracks_filtered.append(filter_track(df_s))

df_f = pd.concat(tracks_filtered, ignore_index=True)
df_f = df_f.sort_values("stamp")

# Downsample for plotting
if PLOT_EVERY > 1:
    df_plot = (
        df_f
        .groupby(["drone", "source"], group_keys=False)
        .apply(lambda g: g.sort_values("stamp").iloc[::PLOT_EVERY])
    )
else:
    df_plot = df_f

# ---------------- TIME NORMALIZATION ----------------
ekf_all = df_plot[df_plot["source"] == "ekf"].copy()
if ekf_all.empty:
    raise RuntimeError("No EKF samples found in CSV.")

# global reference time (first EKF stamp)
t0 = ekf_all["stamp"].min()

# relative time from t0
ekf_all = ekf_all.assign(t_rel=lambda d: d["stamp"] - t0)

# drop early warm-up samples
ekf_all = ekf_all[ekf_all["t_rel"] >= WARMUP_S]
if ekf_all.empty:
    raise RuntimeError(
        "No EKF samples left after warm-up filtering. "
        "Reduce WARMUP_S or check your data."
    )

t_rel_min = ekf_all["t_rel"].min()
t_rel_max = ekf_all["t_rel"].max()
if t_rel_max <= t_rel_min:
    t_rel_max = t_rel_min + 1e-3  # avoid zero span

norm = plt.Normalize(t_rel_min, t_rel_max)
cmap = plt.get_cmap("viridis")

# ------------- PLOT -------------
fig, ax = plt.subplots(figsize=(4.8, 4.8), dpi=300)

global_x = []
global_y = []

for drone in sorted(df_plot["drone"].unique()):
    d_drone = df_plot[df_plot["drone"] == drone]

    # --- RAW ODOM (thin, grey, dashed) ---
    raw = d_drone[d_drone["source"] == "raw"].sort_values("stamp")
    if not raw.empty:
        ax.plot(
            raw["x"].values,
            raw["y"].values,
            linestyle="--",
            linewidth=0.5,
            color=RAW_COLOR,
            alpha=0.6,
            label="odom",
        )
        global_x.append(raw["x"].values)
        global_y.append(raw["y"].values)

    # --- EKF ODOM (time-colored line) ---
    ekf = d_drone[d_drone["source"] == "ekf"].sort_values("stamp")

    # apply same warm-up cut per drone
    t_cut = t0 + WARMUP_S
    ekf = ekf[ekf["stamp"] >= t_cut]

    if len(ekf) >= 2:
        ekf = ekf.merge(
            ekf_all[["stamp", "t_rel"]],
            on="stamp",
            how="left",
            suffixes=("", "_global"),
        )

        x = ekf["x"].to_numpy()
        y = ekf["y"].to_numpy()
        t_rel = ekf["t_rel"].to_numpy()

        global_x.append(x)
        global_y.append(y)

        points = np.column_stack([x, y])
        segments = np.stack([points[:-1], points[1:]], axis=1)
        t_mid = 0.5 * (t_rel[:-1] + t_rel[1:])

        lc = LineCollection(
            segments,
            cmap=cmap,
            norm=norm,
            linewidth=1.2,
            alpha=0.9,
            label="EKF",
        )
        lc.set_array(t_mid)
        ax.add_collection(lc)

        base_color = DRONE_COLORS.get(drone, "#333333")
        ax.scatter(
            x[0], y[0],
            marker="o",
            s=20,
            facecolors="none",
            edgecolors=base_color,
            linewidths=1.0,
            #label=f"{drone} start",
            label="start",
            zorder=3,
        )
        ax.scatter(
            x[-1], y[-1],
            marker="s",
            s=22,
            facecolors=base_color,
            edgecolors="k",
            linewidths=0.6,
            label="end",
            zorder=3,
        )

# ----- limits & style -----
if global_x and global_y:
    gx = np.concatenate(global_x)
    gy = np.concatenate(global_y)
    x_min, x_max = gx.min(), gx.max()
    y_min, y_max = gy.min(), gy.max()

    x_pad = 0.05 * (x_max - x_min if x_max > x_min else 1.0)
    y_pad = 0.05 * (y_max - y_min if y_max > y_min else 1.0)

    ax.set_xlim(x_min - x_pad, x_max + x_pad)
    ax.set_ylim(y_min - y_pad, y_max + y_pad)

ax.xaxis.set_major_locator(MultipleLocator(X_TICK_STEP))
ax.yaxis.set_major_locator(MultipleLocator(Y_TICK_STEP))

ax.set_aspect("equal", adjustable="box")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
# ax.set_title("Drone XY trajectories (time-colored EKF)", pad=8)
ax.grid(True, linewidth=0.5, alpha=0.35)

for spine in ["top", "right"]:
    ax.spines[spine].set_visible(False)

# ----- legend -----
handles, labels = ax.get_legend_handles_labels()
seen = set()
h_clean, l_clean = [], []
for h, l in zip(handles, labels):
    if l in seen:
        continue
    seen.add(l)
    h_clean.append(h)
    l_clean.append(l)

ax.legend(
    h_clean,
    l_clean,
    loc="upper right",
    bbox_to_anchor=(1.02, 1.0),
    borderaxespad=0.3,
    frameon=True,
    framealpha=0.95,
    handlelength=1.3,
    handletextpad=0.4,
    labelspacing=0.3,
    fontsize=5,
)

# ----- colorbar for time -----
cbar = fig.colorbar(
    plt.cm.ScalarMappable(norm=norm, cmap=cmap),
    ax=ax,
    fraction=0.03,
    pad=0.02,
    shrink=0.7,
)
cbar.set_label("time[s]", fontsize=6)
cbar.ax.tick_params(labelsize=5)

fig.tight_layout()
# fig.savefig("multi_drone_xy_time_paper_big.pdf", bbox_inches="tight")
# fig.savefig("multi_drone_xy_time_paper_big.png", dpi=300, bbox_inches="tight")

plt.show()
