#!/usr/bin/env python3
"""
Plots from prune_events*.csv (auto-detected in current directory):
  A) Global cumulative prunes over time + Per-drone cumulative prunes over time (one figure)

Adds:
- Compress a long "no-events" interval (e.g., 150s..340s) on the x-axis
- Mark the break with small zigzag slashes on the x-axis (no SS text)
"""

import os
import glob
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# =========================
# Palettes you chose
# =========================
CONTAINER_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]  # kept (not used in this plot)
DRONE_PALETTE     = ["#0072B2", "#E69F00", "#009E73"]                       # drone1..drone3

# =========================
# Font sizes (EDIT THESE)
# =========================
TITLE_FS   = 12
LABEL_FS   = 14
TICK_FS    = 10
LEGEND_FS  = 10
LEGEND_TITLE_FS = 8

# =========================
# Time-gap compression (EDIT THESE)
# =========================
GAP_START_SEC   = 150.0   # start of the long flat part (original time axis)
GAP_END_SEC     = 340.0   # end of the long flat part (original time axis)
GAP_DISPLAY_SEC = 10.0    # how wide that gap should appear after compression

def apply_fonts():
    plt.rcParams.update({
        "axes.titlesize": TITLE_FS,
        "axes.labelsize": LABEL_FS,
        "xtick.labelsize": TICK_FS,
        "ytick.labelsize": TICK_FS,
        "legend.fontsize": LEGEND_FS,
        "legend.title_fontsize": LEGEND_TITLE_FS,
    })

def drone_sort_key(s: str) -> int:
    m = re.search(r"drone\s*(\d+)", str(s), flags=re.I)
    return int(m.group(1)) if m else 999

def build_map(keys_in_order, palette):
    colors = [palette[i % len(palette)] for i in range(len(keys_in_order))]
    return dict(zip(keys_in_order, colors))

def load_prune_events_csv() -> pd.DataFrame:
    pattern = "*prune_events*.csv"
    files = sorted(glob.glob(pattern))

    if not files:
        files = sorted([f for f in glob.glob("*.csv") if "prune_events" in os.path.basename(f).lower()])

    if not files:
        raise RuntimeError(f"No CSV files found matching '{pattern}' (case-insensitive) in current directory.")

    print(f"[INFO] Found {len(files)} prune_events file(s):")
    for f in files:
        print("  -", f)

    dfs = []
    for f in files:
        d = pd.read_csv(f)
        d["__source_file__"] = os.path.basename(f)
        dfs.append(d)

    df = pd.concat(dfs, ignore_index=True)

    df["stamp_ns"] = pd.to_numeric(df.get("stamp_ns"), errors="coerce")
    df = df[pd.notna(df["stamp_ns"])].copy()
    df.sort_values("stamp_ns", inplace=True)
    df.reset_index(drop=True, inplace=True)
    return df

def explode_drones_seen(df: pd.DataFrame) -> pd.DataFrame:
    d = df.copy()
    d["drones_seen"] = d.get("drones_seen", "").fillna("").astype(str)

    d["drones_seen"] = (
        d["drones_seen"]
        .str.replace(";", ",", regex=False)
        .str.replace("|", ",", regex=False)
        .str.replace(" ", ",", regex=False)
    )
    d["drones_seen_list"] = d["drones_seen"].str.split(",")

    d = d.explode("drones_seen_list")
    d["drone"] = d["drones_seen_list"].astype(str).str.strip()
    d = d.drop(columns=["drones_seen_list"])
    d = d[d["drone"] != ""]
    return d

def compress_time_axis(t: np.ndarray, gap_start: float, gap_end: float, gap_display: float):
    """
    Compress the interval [gap_start, gap_end] so it visually becomes 'gap_display' seconds wide.
    """
    t = np.asarray(t, dtype=float)
    gap = float(gap_end - gap_start)
    if gap <= 0:
        return t.copy(), None

    shift = gap - float(gap_display)
    if shift <= 0:
        return t.copy(), None

    t_plot = t.copy()

    inside = (t >= gap_start) & (t <= gap_end)
    t_plot[inside] = gap_start

    after = (t > gap_end)
    t_plot[after] = t_plot[after] - shift

    info = {
        "gap_start_plot": gap_start,
        "gap_end_plot": gap_end - shift,
    }
    return t_plot, info

def draw_zigzag_break(ax, x_left: float, x_right: float, size=0.015, lw=1.6):
    """
    Draw two small diagonal slashes on the x-axis area (axes coords),
    centered at the compressed gap boundaries.
    """
    # Convert x data coords -> axes coords (so it stays visually consistent)
    xL_ax = ax.transAxes.inverted().transform(ax.transData.transform((x_left, 0)))[0]
    xR_ax = ax.transAxes.inverted().transform(ax.transData.transform((x_right, 0)))[0]

    # y position in axes coords near bottom
    y = 0.02

    # two slashes near left boundary
    ax.plot([xL_ax - size, xL_ax + size], [y - size, y + size],
            transform=ax.transAxes, color="black", lw=lw, clip_on=False)
    ax.plot([xL_ax - size, xL_ax + size], [y + size, y + 3*size],
            transform=ax.transAxes, color="black", lw=lw, clip_on=False)

    # two slashes near right boundary
    ax.plot([xR_ax - size, xR_ax + size], [y - size, y + size],
            transform=ax.transAxes, color="black", lw=lw, clip_on=False)
    ax.plot([xR_ax - size, xR_ax + size], [y + size, y + 3*size],
            transform=ax.transAxes, color="black", lw=lw, clip_on=False)

def main():
    apply_fonts()
    df = load_prune_events_csv()

    df["stamp_ns"] = pd.to_numeric(df.get("stamp_ns"), errors="coerce")
    if df["stamp_ns"].isna().all():
        raise RuntimeError("stamp_ns missing or not numeric.")

    # Time axis: seconds from first stamp
    t0 = df["stamp_ns"].min()
    df["t_sec"] = (df["stamp_ns"] - t0) * 1e-9

    # Explode by drone (drones_seen)
    dfx = explode_drones_seen(df)

    drones = sorted(dfx["drone"].dropna().unique().tolist(), key=drone_sort_key)
    drone_color_map = build_map(drones, DRONE_PALETTE)

    # =========================
    # ONE figure: Global + Per-drone cumulative
    # =========================
    fig, ax = plt.subplots(figsize=(9.5, 5.2))

    # Per-drone curves first (so global can be on top)
    for dr in drones:
        sub = dfx[dfx["drone"] == dr].sort_values("t_sec")
        if sub.empty:
            continue

        t_plot, _ = compress_time_axis(sub["t_sec"].to_numpy(), GAP_START_SEC, GAP_END_SEC, GAP_DISPLAY_SEC)
        cum_d = np.arange(1, len(sub) + 1)

        ax.plot(
            t_plot, cum_d,
            linewidth=1.8,
            alpha=0.95,
            color=drone_color_map[dr],
            label=dr,
            zorder=2
        )

    # Global curve last
    g = df.sort_values("t_sec")
    t_plot_g, info_g = compress_time_axis(g["t_sec"].to_numpy(), GAP_START_SEC, GAP_END_SEC, GAP_DISPLAY_SEC)
    cum_global = np.arange(1, len(g) + 1)

    ax.plot(
        t_plot_g, cum_global,
        linewidth=3.0,
        color="black",
        linestyle="--",
        label="Global (all drones)",
        zorder=10
    )

    ax.grid(True, alpha=0.3)
    ax.tick_params(axis="both", labelsize=TICK_FS)
    ax.legend(title="Curve", loc="best")

    ax.set_xlabel(
        f"Time (s from first stamp)  [gap {GAP_START_SEC:.0f}–{GAP_END_SEC:.0f}s compressed]"
    )
    ax.set_ylabel("Cumulative count")
    ax.set_title("Cumulative pruned tracks over time (gap compressed)")

    # Draw zigzag break marks (no SS text)
    if info_g is not None:
        draw_zigzag_break(ax, info_g["gap_start_plot"], info_g["gap_end_plot"], size=0.012, lw=1.5)

    fig.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
