# pip install pandas matplotlib seaborn
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.lines import Line2D
from pathlib import Path
from typing import List
import glob, re
import numpy as np

# ========== CONFIG ==========
# LOG_DIR   = Path(r"F:\My Papers\Ship and drones\data\logs\detection")  # Windows example
LOG_DIR = Path("/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs/")

FILE_GLOB = "detections*drone*.csv"    # read ALL matching files
RECURSIVE = False                      # set True to search subfolders
SAVE_PNG  = False                      # save figures next to the first CSV
PLOT_MODE = "box"                      # "scatter" or "box"
BOX_WIDTH = 0.4                        # for box plots (try 0.3–0.5)

# Colors: choose palette name OR custom list (order is C1, C2, C3, ...)
#PALETTE = "tab10"
CONTAINER_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
DRONE_PALETTE = ["#0072B2", "#E69F00", "#009E73"]   # blue, orange, green

# NEW: Font sizes (edit these)
BASE_FONT      = 12
AXIS_LABEL_FS  = 10
TICK_LABEL_FS  = 10
LEGEND_FS      = 8
LEGEND_TITLE_FS = 8
# ============================

def list_csvs(folder: Path, pattern: str, recursive: bool = False) -> List[Path]:
    if recursive:
        files = glob.glob(str(folder / "**" / pattern), recursive=True)
    else:
        files = glob.glob(str(folder / pattern))
    paths = [Path(f) for f in files]
    if not paths:
        raise FileNotFoundError(f"No CSV matched pattern '{pattern}' in {folder}")
    paths.sort()
    return paths

def coerce_numeric(series: pd.Series, name: str) -> pd.Series:
    s = series.astype(str).str.strip().str.replace(r"[^\d\.\-eE+]", "", regex=True)
    out = pd.to_numeric(s, errors="coerce")
    out = out.fillna(pd.to_numeric(series, errors="coerce"))
    if out.isna().any():
        raise ValueError(f"Column '{name}' still has {out.isna().sum()} non-numeric rows after cleaning.")
    return out

def extract_drone(filename: str) -> str:
    m = re.search(r"(drone\d+)", filename, flags=re.I)
    return m.group(1).lower() if m else Path(filename).stem

def drone_sort_key(name: str) -> int:
    m = re.search(r"\d+", str(name))
    return int(m.group()) if m else 0

def infer_epoch_unit(series_numeric: pd.Series) -> str:
    m = series_numeric.dropna().astype(float).abs().median()
    if m > 1e16: return "ns"
    if m > 1e13: return "us"
    if m > 1e11: return "ms"
    return "s"

def make_time_from_stamp(df: pd.DataFrame) -> pd.Series:
    if "stamp" not in df.columns:
        return pd.Series(pd.RangeIndex(len(df)), index=df.index, name="time")

    stamp_raw = df["stamp"]
    t = None

    if "ns" in df.columns:
        secs = pd.to_numeric(stamp_raw, errors="coerce")
        ns = pd.to_numeric(df["ns"], errors="coerce").fillna(0)
        if secs.notna().any():
            t = pd.to_datetime(secs, unit="s", utc=True) + pd.to_timedelta(ns, unit="ns")

    if t is None:
        parsed = pd.to_datetime(stamp_raw, errors="coerce", utc=True)
        if parsed.notna().sum() >= len(df) * 0.8:
            t = parsed
        else:
            sn = pd.to_numeric(stamp_raw, errors="coerce")
            unit = infer_epoch_unit(sn)
            t = pd.to_datetime(sn, unit=unit, utc=True)

    try:
        t = t.dt.tz_convert("Europe/Paris")
    except Exception:
        pass

    return t.rename("time")

def gt_display_label(gt: str) -> str:
    """container1 -> C1, container2 -> C2, ..."""
    m = re.search(r"container\s*(\d+)", str(gt), flags=re.I)
    return f"C{m.group(1)}" if m else str(gt)

def build_palette_map(labels_in_order: List[str]):
    """
    Returns dict: label -> color.
    Works for PALETTE as a seaborn palette name OR a list of hex colors.
    """
    colors = sns.color_palette(CONTAINER_PALETTE, n_colors=len(labels_in_order))
    return dict(zip(labels_in_order, colors))

def apply_global_fonts():
    plt.rcParams.update({
        "font.size": BASE_FONT,
        "axes.titlesize": AXIS_LABEL_FS,
        "axes.labelsize": AXIS_LABEL_FS,
        "xtick.labelsize": TICK_LABEL_FS,
        "ytick.labelsize": TICK_LABEL_FS,
        "legend.fontsize": LEGEND_FS,
        "legend.title_fontsize": LEGEND_TITLE_FS,
    })

def style_axis(ax, xlabel=None, ylabel=None, legend_loc="best"):
    if xlabel is not None:
        ax.set_xlabel(xlabel, fontsize=AXIS_LABEL_FS)
    if ylabel is not None:
        ax.set_ylabel(ylabel, fontsize=AXIS_LABEL_FS)

    ax.tick_params(axis="both", labelsize=TICK_LABEL_FS)

    leg = ax.get_legend()
    if leg:
        leg.set_title("")  # no title
        for t in leg.get_texts():
            t.set_fontsize(LEGEND_FS)
        leg.get_title().set_fontsize(LEGEND_TITLE_FS)
        leg._loc = legend_loc  # respect rcParams; alternative is ax.legend(loc=...)

def load_one_csv(p: Path) -> pd.DataFrame:
    df = pd.read_csv(p)
    missing = [c for c in ["score", "err_norm"] if c not in df.columns]
    if missing:
        raise ValueError(f"{p.name}: missing required column(s): {missing}")

    if "gt_name" not in df.columns:
        df["gt_name"] = "unknown"

    df["source"]   = p.name
    df["drone"]    = extract_drone(p.name)
    df["_score"]   = coerce_numeric(df["score"], "score")
    df["_err"]     = coerce_numeric(df["err_norm"], "err_norm")
    df["time"]     = make_time_from_stamp(df)
    df["gt_label"] = df["gt_name"].apply(gt_display_label)

    return df[["gt_name", "gt_label", "_score", "_err", "source", "drone", "time"]]

def main():
    sns.set_theme(style="whitegrid", context="talk")
    apply_global_fonts()

    files = list_csvs(LOG_DIR, FILE_GLOB, RECURSIVE)
    print("Reading files:")
    for f in files:
        print("  -", f)

    frames = [load_one_csv(p) for p in files]
    df = pd.concat(frames, ignore_index=True)

    if PLOT_MODE.lower() == "scatter":
        # ---------- SCATTER (dual y-axes) ----------
        df_sc = df.sort_values("time")

        gt_order    = list(dict.fromkeys(df_sc["gt_label"].astype(str)))
        drone_order = list(dict.fromkeys(df_sc["drone"].astype(str)))

        color_map = build_palette_map(gt_order)

        marker_cycle = ['o', 's', 'D', '^', 'v', 'P', 'X', '*', '<', '>']
        marker_map = {d: marker_cycle[i % len(marker_cycle)] for i, d in enumerate(drone_order)}

        fig_sc, ax1 = plt.subplots(figsize=(13, 6))
        ax2 = ax1.twinx()

        ax2.grid(False)
        ax1.set_axisbelow(True)
        ax2.patch.set_alpha(0)

        for d in drone_order:
            sub = df_sc[df_sc["drone"] == d]

            ax1.scatter(
                sub["time"] if "time" in sub else range(len(sub)),
                sub["_score"],
                s=36,
                marker=marker_map[d],
                c=[color_map[str(g)] for g in sub["gt_label"].astype(str)],
                edgecolors="none"
            )
            ax2.scatter(
                sub["time"] if "time" in sub else range(len(sub)),
                sub["_err"],
                s=36,
                marker=marker_map[d],
                c=[color_map[str(g)] for g in sub["gt_label"].astype(str)],
                edgecolors="none"
            )

        xlabel = "" if "time" in df_sc and getattr(df_sc["time"].dt, "tz", None) else "time / index"
        ax1.set_xlabel(xlabel, fontsize=AXIS_LABEL_FS)
        ax1.set_ylabel("score", fontsize=AXIS_LABEL_FS)
        ax2.set_ylabel("err_norm", fontsize=AXIS_LABEL_FS)

        ax1.tick_params(axis="both", labelsize=TICK_LABEL_FS)
        ax2.tick_params(axis="both", labelsize=TICK_LABEL_FS)

        # Legends (custom) with font sizes
        color_handles = [
            Line2D([0], [0], marker='o', linestyle='',
                   markerfacecolor=color_map[g], markeredgecolor=color_map[g], markersize=8,
                   label=g)
            for g in gt_order
        ]
        leg1 = ax1.legend(handles=color_handles, title="", loc="upper left", fontsize=LEGEND_FS)
        ax1.add_artist(leg1)

        marker_handles = [
            Line2D([0], [0], marker=marker_map[d], linestyle='', color='black', label=d)
            for d in drone_order
        ]
        ax1.legend(handles=marker_handles, title="", loc="upper right", fontsize=LEGEND_FS)

        plt.tight_layout()
        if SAVE_PNG:
            out = files[0].with_suffix("")
            png_sc = out.parent / f"{out.name}_ALL_scatter_dualaxes_color_gt_marker_drone.png"
            plt.savefig(png_sc, dpi=150, bbox_inches="tight"); print(f"Saved figure: {png_sc}")
        plt.show()

    elif PLOT_MODE.lower() == "box":
        # ---------- BOX PLOTS (two figures total) ----------
        x_order   = sorted(df["drone"].astype(str).unique(), key=drone_sort_key)
        hue_order = list(dict.fromkeys(df["gt_label"].astype(str)))

        palette_map = build_palette_map(hue_order)

        # Figure 1: err_norm
        fig1, ax1 = plt.subplots(figsize=(14, 6))
        sns.boxplot(
            data=df, x="drone", y="_err",
            hue="gt_label", order=x_order, hue_order=hue_order,
            palette=palette_map,
            width=BOX_WIDTH, dodge=True, linewidth=1, ax=ax1
        )
        style_axis(ax1, xlabel="drone", ylabel="err_norm")
        ax1.legend(title="", loc="best", fontsize=LEGEND_FS)
        plt.tight_layout()
        if SAVE_PNG:
            out = files[0].with_suffix("")
            png1 = out.parent / f"{out.name}_PERDRONE_box_errnorm.png"
            plt.savefig(png1, dpi=150, bbox_inches="tight"); print(f"Saved figure: {png1}")
        plt.show()

        # Figure 2: score
        fig2, ax2 = plt.subplots(figsize=(14, 6))
        sns.boxplot(
            data=df, x="drone", y="_score",
            hue="gt_label", order=x_order, hue_order=hue_order,
            palette=palette_map,
            width=BOX_WIDTH, dodge=True, linewidth=1, ax=ax2
        )
        style_axis(ax2, xlabel="drone", ylabel="score")
        ax2.legend(title="", loc="best", fontsize=LEGEND_FS)
        plt.tight_layout()
        if SAVE_PNG:
            out = files[0].with_suffix("")
            png2 = out.parent / f"{out.name}_PERDRONE_box_score.png"
            plt.savefig(png2, dpi=150, bbox_inches="tight"); print(f"Saved figure: {png2}")
        plt.show()

    else:
        raise ValueError("PLOT_MODE must be 'scatter' or 'box'.")

if __name__ == "__main__":
    main()
