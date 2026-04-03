#!/usr/bin/env python3
"""
plot_assignment_with_errors_modes.py

Reads ALL CSV files in the CURRENT FOLDER (or a provided --glob), merges them, and displays:

A) Per-drone / per-container ASSIGNMENT matrix (heatmap)
   value = fraction of assigned rows where that drone's primary_class_id matches the container

B) Per-drone time-series panels (with TRACKING vs SURVEILLANCE background shading):
   1) Assigned container over time (categorical encoded as integer)
   2) XY position error vs GT over time (err_xy), with at_circle markers
   3) Uncertainty proxy target_logdetP (and post_logdetP if available), with at_circle markers

C) Summary bars (inside vs outside hover ring):
   - Δ err_xy (outside - inside)  (positive => lower error inside ring)
   - Δ logdetP (outside - inside) (positive => lower uncertainty inside ring)

Notes / assumptions:
- CSV produced by assignment_node logger includes:
  primary_class_id, at_circle, target_logdetP (and optionally post_logdetP),
  and either (err_x, err_y) OR (target_x,target_y,gt_x,gt_y with has_gt==1).
- For TRACKING/SURVEILLANCE shading:
  - If 'mode' column exists, it is used.
  - Else it is inferred from is_assigned.
  - If 'latched_surveillance' exists and == 1, forced to SURVEILLANCE.

Run:
  python3 plot_assignment_with_errors_modes.py
Or:
  python3 plot_assignment_with_errors_modes.py --glob "assignment*.csv"
"""

import os
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# ---------------------------- Helpers ---------------------------- #

def to_numeric(df: pd.DataFrame, cols):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")


def build_time_seconds(df: pd.DataFrame) -> pd.Series:
    """
    Prefer t_sec. Else build from stamp_sec+stamp_nsec. Else seq. Else index.
    """
    if "t_sec" in df.columns:
        return pd.to_numeric(df["t_sec"], errors="coerce")

    if "stamp_sec" in df.columns and "stamp_nsec" in df.columns:
        sec = pd.to_numeric(df["stamp_sec"], errors="coerce")
        nsec = pd.to_numeric(df["stamp_nsec"], errors="coerce")
        t = sec + 1e-9 * nsec
        t0 = t.min()
        if pd.notna(t0):
            t = t - t0
        return t

    if "seq" in df.columns:
        return pd.to_numeric(df["seq"], errors="coerce")

    return pd.Series(df.index, index=df.index, dtype=float)


def rolling_mean(s: pd.Series, win: int) -> pd.Series:
    if win <= 1:
        return s
    return s.rolling(window=win, min_periods=max(1, win // 3)).mean()


def ensure_drone_ns(df: pd.DataFrame) -> pd.DataFrame:
    if "drone_ns" not in df.columns:
        if "drone_idx" in df.columns:
            df["drone_ns"] = df["drone_idx"].apply(
                lambda x: f"/drone{int(x)+1}" if pd.notna(x) else "/drone?"
            )
        else:
            df["drone_ns"] = "/drone?"
    return df


def infer_container_labeler(primary_class_ids: np.ndarray):
    """
    Tries to generate human-friendly labels:
    - If values look like 0..K-1, label as container1..containerK
    - If values look like 1..K, label as container1..containerK
    Otherwise label as class<ID>
    """
    vals = sorted({int(v) for v in primary_class_ids if np.isfinite(v) and int(v) != -1})
    if not vals:
        def label(cid: int) -> str:
            return "unassigned" if cid == -1 else f"class{cid}"
        return label

    vmin, vmax = min(vals), max(vals)
    if vmin == 0 and vmax <= 50:
        def label(cid: int) -> str:
            if cid == -1:
                return "unassigned"
            return f"container{cid+1}"
        return label

    if vmin == 1 and vmax <= 50:
        def label(cid: int) -> str:
            if cid == -1:
                return "unassigned"
            return f"container{cid}"
        return label

    def label(cid: int) -> str:
        return "unassigned" if cid == -1 else f"class{cid}"
    return label


def compute_err_xy(df: pd.DataFrame) -> pd.Series:
    """
    Priority:
      1) if err_x and err_y exist -> sqrt(err_x^2 + err_y^2)
      2) else if target_x,target_y,gt_x,gt_y exist -> sqrt((target_x-gt_x)^2 + (target_y-gt_y)^2) when has_gt==1
      3) else -> NaN
    """
    if "err_x" in df.columns and "err_y" in df.columns:
        ex = pd.to_numeric(df["err_x"], errors="coerce")
        ey = pd.to_numeric(df["err_y"], errors="coerce")
        return np.sqrt(ex * ex + ey * ey)

    needed = {"target_x", "target_y", "gt_x", "gt_y"}
    if needed.issubset(df.columns):
        tx = pd.to_numeric(df["target_x"], errors="coerce")
        ty = pd.to_numeric(df["target_y"], errors="coerce")
        gx = pd.to_numeric(df["gt_x"], errors="coerce")
        gy = pd.to_numeric(df["gt_y"], errors="coerce")
        err = np.sqrt((tx - gx) ** 2 + (ty - gy) ** 2)

        if "has_gt" in df.columns:
            has = pd.to_numeric(df["has_gt"], errors="coerce").fillna(0).astype(int)
            err = err.where(has == 1, np.nan)

        return err

    return pd.Series(np.nan, index=df.index)


def load_and_merge(glob_pattern: str) -> pd.DataFrame:
    files = sorted(glob.glob(glob_pattern))
    files = [f for f in files if os.path.isfile(f)]
    if not files:
        raise RuntimeError(f"No CSV files found matching '{glob_pattern}' in current folder.")

    dfs = []
    for f in files:
        try:
            d = pd.read_csv(f)
            d["__source_file__"] = os.path.basename(f)
            dfs.append(d)
        except Exception as e:
            print(f"[WARN] Skipping {f}: {e}")

    if not dfs:
        raise RuntimeError("No readable CSV files found.")
    return pd.concat(dfs, ignore_index=True)


# ---------------------------- Mode shading helpers ---------------------------- #

def normalize_mode_series(sub: pd.DataFrame) -> pd.Series:
    """
    Returns a clean mode series: 'TRACKING' or 'SURVEILLANCE'
    Priority:
      - if 'mode' exists, use it
      - else infer from 'is_assigned' (assigned => TRACKING else SURVEILLANCE)
      - if 'latched_surveillance' exists and == 1 => SURVEILLANCE
    """
    if "mode" in sub.columns:
        m = sub["mode"].fillna("").astype(str).str.upper().str.strip()
        m = m.replace({"": "SURVEILLANCE"})
    else:
        m = np.where(sub.get("is_assigned", False), "TRACKING", "SURVEILLANCE")
        m = pd.Series(m, index=sub.index)

    if "latched_surveillance" in sub.columns:
        lat = pd.to_numeric(sub["latched_surveillance"], errors="coerce").fillna(0).astype(int)
        m = m.where(lat == 0, "SURVEILLANCE")

    # normalize potential variants
    m = m.replace({
        "TRACK": "TRACKING",
        "SURVEIL": "SURVEILLANCE",
    })
    m = m.where(m.isin(["TRACKING", "SURVEILLANCE"]), "SURVEILLANCE")
    return m


def contiguous_segments(t: np.ndarray, mode: np.ndarray):
    """
    Given time array and mode array (strings), return list of (mode, t_start, t_end).
    t_end is the last sample time in that segment.
    """
    if len(t) == 0:
        return []
    segs = []
    start = 0
    for i in range(1, len(t)):
        if mode[i] != mode[i - 1]:
            segs.append((mode[i - 1], t[start], t[i - 1]))
            start = i
    segs.append((mode[-1], t[start], t[-1]))
    return segs


def shade_mode(ax, t: np.ndarray, mode: np.ndarray, alpha=0.10):
    """
    Shade background by mode.
    Avoid explicit hard-coded colors; uses matplotlib default prop_cycle.
    TRACKING: solid span
    SURVEILLANCE: dotted hatch span
    """
    segs = contiguous_segments(t, mode)

    cycle = plt.rcParams["axes.prop_cycle"].by_key().get("color", ["0.7", "0.9"])
    c_track = cycle[0]
    c_surve = cycle[1] if len(cycle) > 1 else cycle[0]

    for m, a, b in segs:
        if not np.isfinite(a) or not np.isfinite(b) or a == b:
            continue
        if m == "TRACKING":
            ax.axvspan(a, b, alpha=alpha, color=c_track, linewidth=0)
        else:
            ax.axvspan(a, b, alpha=alpha, color=c_surve, hatch="..", linewidth=0)


def add_mode_switch_lines(ax, t: np.ndarray, mode: np.ndarray, alpha=0.25):
    """
    Optional: draw vertical lines at mode change boundaries.
    """
    if len(t) < 2:
        return
    for i in range(1, len(t)):
        if mode[i] != mode[i - 1]:
            ax.axvline(t[i], linestyle="--", linewidth=1, alpha=alpha)


# ---------------------------- Main plotting ---------------------------- #

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default="*.csv", help="CSV glob pattern in current folder (default: *.csv)")
    ap.add_argument("--roll", type=int, default=10, help="Rolling mean window (samples). 1 disables.")
    ap.add_argument("--only_assigned", action="store_true", help="Plot time-series only for assigned rows.")
    ap.add_argument("--switch_lines", action="store_true", help="Draw dashed vertical lines at mode switches.")
    args = ap.parse_args()

    print("Current directory:", os.getcwd())
    print(f"Loading CSVs with glob: {args.glob}")
    df = load_and_merge(args.glob)

    # Numeric columns
    to_numeric(df, [
        "seq", "t_sec", "stamp_sec", "stamp_nsec",
        "drone_idx",
        "primary_class_id",
        "primary_target_id",
        "primary_track_idx",
        "dopt_gain", "dopt",
        "dist", "cost",
        "target_logdetP", "post_logdetP",
        "at_circle",
        "has_gt",
        "gt_x", "gt_y", "gt_z",
        "err_x", "err_y", "err_z", "err_norm",
        "drone_x", "drone_y", "drone_z",
        "target_x", "target_y", "target_z",
        "latched_surveillance",
    ])

    df = ensure_drone_ns(df)
    df["time_s"] = build_time_seconds(df)

    if "primary_class_id" not in df.columns:
        raise RuntimeError("Missing 'primary_class_id' in CSVs.")

    df["primary_class_id"] = df["primary_class_id"].fillna(-1).astype(int)
    df["is_assigned"] = df["primary_class_id"] != -1

    if "at_circle" in df.columns:
        df["at_circle"] = df["at_circle"].fillna(0).astype(int)
    else:
        df["at_circle"] = 0

    df = df[pd.notna(df["time_s"])].copy()
    df = df.sort_values(["drone_ns", "time_s"])

    # XY error
    df["err_xy"] = compute_err_xy(df)

    # Container labeling
    labeler = infer_container_labeler(df["primary_class_id"].to_numpy())
    df["container_label"] = df["primary_class_id"].apply(labeler)

    drones = sorted(df["drone_ns"].dropna().unique().tolist())

    containers_present = sorted([c for c in df["primary_class_id"].unique().tolist() if c != -1])
    if len(containers_present) == 0:
        containers_present = [0, 1, 2, 3, 4]
    container_labels = [labeler(c) for c in containers_present]

    # ------------------ (A) Assignment Matrix ------------------ #
    mat = np.full((len(drones), len(containers_present)), np.nan, dtype=float)

    for i, dn in enumerate(drones):
        sub = df[df["drone_ns"] == dn]
        assigned = sub[sub["is_assigned"]]
        denom = len(assigned)
        if denom == 0:
            mat[i, :] = 0.0
            continue
        for j, cid in enumerate(containers_present):
            mat[i, j] = (assigned["primary_class_id"] == cid).sum() / denom

    fig1 = plt.figure()
    axm = fig1.add_subplot(111)
    im = axm.imshow(mat, aspect="auto")
    #axm.set_title("Per-drone / per-container assignment ")
    axm.set_xlabel("Container")
    axm.set_ylabel("Drone")
    axm.set_xticks(range(len(container_labels)))
    axm.set_xticklabels(container_labels, rotation=30, ha="right")
    axm.set_yticks(range(len(drones)))
    axm.set_yticklabels(drones)
    plt.colorbar(im, ax=axm, fraction=0.046, pad=0.04, label="fraction of assigned samples")
    for r in range(mat.shape[0]):
        for c in range(mat.shape[1]):
            axm.text(c, r, f"{mat[r, c]:.2f}", ha="center", va="center", fontsize=9)

    # ------------------ (B) Per-drone panels ------------------ #
    ncols = max(1, len(drones))
    fig2 = plt.figure(figsize=(5 * ncols, 11))
    gs = fig2.add_gridspec(3, ncols)

    inside_stats = []

    for col, dn in enumerate(drones):
        sub = df[df["drone_ns"] == dn].copy().sort_values("time_s")

        if args.only_assigned:
            sub = sub[sub["is_assigned"]].copy()

        if sub.empty:
            for row in range(3):
                ax = fig2.add_subplot(gs[row, col])
                ax.set_axis_off()
                if row == 1:
                    ax.text(0.5, 0.5, f"{dn}\n(no samples)", ha="center", va="center")
            continue

        t = sub["time_s"].to_numpy()
        atc = sub["at_circle"].fillna(0).astype(int).to_numpy() == 1

        mode_arr = normalize_mode_series(sub).to_numpy()

        # --- Row 0: Assigned container id
        ax0 = fig2.add_subplot(gs[0, col])
        shade_mode(ax0, t, mode_arr)
        if args.switch_lines:
            add_mode_switch_lines(ax0, t, mode_arr)
        y = sub["primary_class_id"].to_numpy()
        ax0.plot(t, y, linewidth=1)
        if atc.any():
            ax0.scatter(t[atc], y[atc], marker="x", s=35, linewidths=1.5, label="at_circle")
        #ax0.set_title(f"{dn} — assigned container (primary_class_id)")
        ax0.set_xlabel("time (s)")
        ax0.set_ylabel("class_id")
        ax0.grid(True, alpha=0.3)

        yticks = sorted(set(int(v) for v in y if np.isfinite(v)))
        if -1 not in yticks:
            yticks = [-1] + yticks
        ax0.set_yticks(yticks)
        ax0.set_yticklabels([labeler(v) for v in yticks])
        if atc.any():
            ax0.legend(loc="best")

        # --- Row 1: XY error
        ax1p = fig2.add_subplot(gs[1, col])
        shade_mode(ax1p, t, mode_arr)
        if args.switch_lines:
            add_mode_switch_lines(ax1p, t, mode_arr)
        err_xy = rolling_mean(pd.to_numeric(sub["err_xy"], errors="coerce"), args.roll)
        ax1p.plot(t, err_xy, linewidth=1)
        if atc.any():
            ax1p.scatter(t[atc], err_xy.to_numpy()[atc], marker="x", s=35, linewidths=1.5, label="at_circle")
        #ax1p.set_title(f"{dn} — XY error vs GT (m), rolling avg (win={args.roll})")
        ax1p.set_xlabel("time (s)")
        ax1p.set_ylabel("err_xy (m)")
        ax1p.grid(True, alpha=0.3)
        if atc.any():
            ax1p.legend(loc="best")

        # --- Row 2: Uncertainty proxy logdet(P)
        ax2p = fig2.add_subplot(gs[2, col])
        shade_mode(ax2p, t, mode_arr)
        if args.switch_lines:
            add_mode_switch_lines(ax2p, t, mode_arr)
        if "target_logdetP" in sub.columns:
            ld = rolling_mean(pd.to_numeric(sub["target_logdetP"], errors="coerce"), args.roll)
            ax2p.plot(t, ld, linewidth=1, label="target_logdetP (avg)")
        else:
            ld = pd.Series(np.nan, index=sub.index)

        if "post_logdetP" in sub.columns:
            pld = rolling_mean(pd.to_numeric(sub["post_logdetP"], errors="coerce"), args.roll)
            ax2p.plot(t, pld, linewidth=1, label="post_logdetP (avg)")

        if atc.any() and "target_logdetP" in sub.columns:
            ax2p.scatter(t[atc], ld.to_numpy()[atc], marker="x", s=35, linewidths=1.5, label="at_circle")

        #ax2p.set_title(f"{dn} — uncertainty proxy (lower is better), rolling avg (win={args.roll})")
        ax2p.set_xlabel("time (s)")
        ax2p.set_ylabel("logdet(P)")
        ax2p.grid(True, alpha=0.3)
        ax2p.legend(loc="best")

        # --- Inside/outside stats (assigned only)
        sub_assigned = sub[sub["is_assigned"]].copy()
        if not sub_assigned.empty:
            atc2 = sub_assigned["at_circle"].fillna(0).astype(int).to_numpy() == 1
            in_df = sub_assigned[atc2]
            out_df = sub_assigned[~atc2]

            def mean_no_nan(series):
                s = pd.to_numeric(series, errors="coerce").to_numpy()
                return float(np.nanmean(s)) if np.isfinite(s).any() else np.nan

            inside_stats.append({
                "drone": dn,
                "n_in": int(len(in_df)),
                "n_out": int(len(out_df)),
                "err_in": mean_no_nan(in_df["err_xy"]) if "err_xy" in in_df.columns else np.nan,
                "err_out": mean_no_nan(out_df["err_xy"]) if "err_xy" in out_df.columns else np.nan,
                "ld_in": mean_no_nan(in_df["target_logdetP"]) if "target_logdetP" in in_df.columns else np.nan,
                "ld_out": mean_no_nan(out_df["target_logdetP"]) if "target_logdetP" in out_df.columns else np.nan,
            })

    #fig2.suptitle("Per-drone: assigned container, XY error, and uncertainty" )
    #fig2.text(
        #0.01, 0.995,
        #"Shading: TRACKING (solid) | SURVEILLANCE (dotted)   "
        #+ ("| switch lines: ON" if args.switch_lines else "| switch lines: OFF"),
        #ha="left", va="top"
    #)

    # ------------------ (C) Summary bar chart ------------------ #
    if inside_stats:
        stats_df = pd.DataFrame(inside_stats)

        fig3 = plt.figure(figsize=(12, 6))
        ax3 = fig3.add_subplot(111)

        x = np.arange(len(stats_df))
        err_drop = stats_df["err_out"] - stats_df["err_in"]  # positive => lower error inside
        ld_drop = stats_df["ld_out"] - stats_df["ld_in"]     # positive => lower logdet inside

        ax3.bar(x - 0.2, err_drop.to_numpy(), width=0.4,
                label="Δ err_xy (outside - inside)  (positive=better inside)")
        ax3.bar(x + 0.2, ld_drop.to_numpy(), width=0.4,
                label="Δ logdetP (outside - inside) (positive=better inside)")

        ax3.set_xticks(x)
        ax3.set_xticklabels(stats_df["drone"].tolist(), rotation=20, ha="right")
        #ax3.set_title("Inside hover ring effect summary (per drone)")
        ax3.set_ylabel("difference (positive supports 'better inside ring')")
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc="best")

        for i in range(len(stats_df)):
            ax3.text(i, 0, f"in={int(stats_df.loc[i, 'n_in'])}, out={int(stats_df.loc[i, 'n_out'])}",
                     ha="center", va="bottom", rotation=90, fontsize=8)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
