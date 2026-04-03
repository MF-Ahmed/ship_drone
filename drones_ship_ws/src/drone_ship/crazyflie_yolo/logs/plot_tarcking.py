# pip install pandas matplotlib seaborn
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import glob, re

# ========= CONFIG =========
#TRACK_DIR  = Path(r"F:\My Papers\Ship and drones\data\logs\tracking") .. for windows 
TRACK_DIR = Path("/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs/")

TRACK_GLOB = "tracking*.csv"
SAVE_PNG   = False
# =========================

def coerce_numeric(series: pd.Series) -> pd.Series:
    s = series.astype(str).str.strip().str.replace(r"[^\d\.\-eE+]", "", regex=True)
    out = pd.to_numeric(s, errors="coerce")
    return out

def load_tracking_csvs(folder: Path, pattern: str="tracking*.csv") -> pd.DataFrame:
    files = sorted(glob.glob(str(folder / pattern)))
    if not files:
        raise FileNotFoundError(f"No tracking CSV matched '{pattern}' in {folder}")

    frames = []
    for fp in files:
        p = Path(fp)
        # read header robustly (handles extra columns)
        with open(p, "r", encoding="utf-8", errors="replace") as f:
            cols = f.readline().rstrip("\n").split(",")
        df = pd.read_csv(p, names=cols, skiprows=1, engine="python", on_bad_lines="skip")

        # numeric coercions if present
        for c in ["stamp","err_norm","est_x","est_y","est_z","gt_x","gt_y","gt_z","track_id"]:
            if c in df.columns:
                df[c] = coerce_numeric(df[c])

        # time from 'stamp' if present
        if "stamp" in df.columns:
            t = pd.to_datetime(df["stamp"], unit="s", utc=True)
            try:
                t = t.dt.tz_convert("Europe/Paris")
            except Exception:
                pass
            df["time"] = t.dt.tz_localize(None)  # tz-naive for matplotlib

        # container name
        if "gt_name" not in df.columns:
            df["gt_name"] = "unknown"

        # drone id: in some tracking files 'ns' actually stores 'drone1/2/3'
        if "ns" in df.columns:
            ns_str = df["ns"].astype(str).str.lower()
            is_drone = ns_str.str.contains(r"drone\d+")
            df["drone"] = ns_str.where(is_drone, None)
        if "drone" not in df.columns or df["drone"].isna().all():
            # fallback to filename
            m = re.search(r"(drone\d+)", p.name, flags=re.I)
            df["drone"] = (m.group(1).lower() if m else "drone?")

        df["source"] = p.name
        frames.append(df)

    return pd.concat(frames, ignore_index=True)

def plot_tracking(tr: pd.DataFrame, save_png=False, out_base: Path=None):
    sns.set_theme(style="whitegrid", context="talk")

    # keep first-seen order
    gt_order    = list(dict.fromkeys(tr["gt_name"].astype(str)))
    drone_order = list(dict.fromkeys(tr["drone"].astype(str)))

    # 1) err_norm vs time (scatter) — color = gt_name, marker = drone
    if {"time","err_norm","gt_name","drone"}.issubset(tr.columns):
        dfp = tr.dropna(subset=["time","err_norm"]).sort_values("time")
        palette = sns.color_palette(n_colors=len(gt_order))
        color_map = dict(zip(gt_order, palette))
        marker_cycle = ['o','s','D','^','v','P','X','*','<','>']
        marker_map = {d: marker_cycle[i % len(marker_cycle)] for i, d in enumerate(drone_order)}

        fig, ax = plt.subplots(figsize=(13, 6))
        for d in drone_order:
            sub = dfp[dfp["drone"] == d]
            ax.scatter(
                sub["time"], sub["err_norm"], s=30,
                marker=marker_map[d],
                c=[color_map[str(g)] for g in sub["gt_name"].astype(str)],
                edgecolors="none"
            )

        ax.set_xlabel("time (Europe/Paris)")
        ax.set_ylabel("err_norm")
        ax.set_title("Tracking — err_norm vs time (color=container, marker=drone)")

        # legends
        from matplotlib.lines import Line2D
        color_handles = [Line2D([0],[0], marker='o', linestyle='',
                                markerfacecolor=color_map[g], markeredgecolor=color_map[g],
                                markersize=8, label=g) for g in gt_order]
        marker_handles = [Line2D([0],[0], marker=marker_map[d], linestyle='',
                                 color='black', label=d) for d in drone_order]
        leg1 = ax.legend(handles=color_handles, title="gt_name (container)", loc="upper left")
        ax.add_artist(leg1)
        ax.legend(handles=marker_handles, title="drone", loc="upper right")

        plt.tight_layout()
        if save_png and out_base is not None:
            plt.savefig(out_base.parent / f"{out_base.stem}_tracking_err_vs_time.png", dpi=150, bbox_inches="tight")
        plt.show()

    # 2) Box plot — err_norm by container, hue = drone (two figures total not needed for tracking)
    if {"err_norm","gt_name","drone"}.issubset(tr.columns):
        fig, ax = plt.subplots(figsize=(14, 6))
        sns.boxplot(
            data=tr, x="gt_name", y="err_norm",
            hue="drone", order=gt_order, hue_order=drone_order,
            width=0.4, dodge=True, linewidth=1, ax=ax
        )
        ax.set_title("Tracking — err_norm by container (hue=drone)")
        ax.set_xlabel("gt_name"); ax.set_ylabel("err_norm")
        ax.legend(title="drone", loc="best")
        plt.tight_layout()
        if save_png and out_base is not None:
            plt.savefig(out_base.parent / f"{out_base.stem}_tracking_box_err_by_container.png", dpi=150, bbox_inches="tight")
        plt.show()

    # 3) XY estimated trajectories per container
    if {"est_x","est_y","gt_name"}.issubset(tr.columns):
        fig, ax = plt.subplots(figsize=(6, 6))
        for name, g in tr.groupby("gt_name"):
            g = g.sort_values("time") if "time" in g.columns else g
            ax.plot(g["est_x"], g["est_y"], marker="o", linewidth=1, markersize=2, label=str(name))
        ax.set_xlabel("est_x"); ax.set_ylabel("est_y")
        ax.set_title("Tracking — XY estimated trajectories by container")
        ax.axis("equal"); ax.legend(loc="best")
        plt.tight_layout()
        if save_png and out_base is not None:
            plt.savefig(out_base.parent / f"{out_base.stem}_tracking_xy_est.png", dpi=150, bbox_inches="tight")
        plt.show()

if __name__ == "__main__":
    tr_df = load_tracking_csvs(TRACK_DIR, TRACK_GLOB)
    print(f"Loaded {len(tr_df)} rows from {TRACK_DIR}")
    # pick a base path for filenames if saving
    out_base = Path(sorted(glob.glob(str(TRACK_DIR / TRACK_GLOB)))[0]) if SAVE_PNG else None
    plot_tracking(tr_df, SAVE_PNG, out_base)
