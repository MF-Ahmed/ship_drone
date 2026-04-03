#!/usr/bin/env python3
"""
Plots from fusion_stats*.csv (auto-detected in current directory):
  1) Global cumulative totals (raw vs pruned) over time
  2) Global pruning efficiency (pruned/raw) over time
  6) Per-drone pruning efficiency (pruned/raw) over time
  9) Final snapshot heatmap: per-class prune ratio per drone (pruned_class_cum/raw_class_cum)
  10) Final snapshot: per-drone RAW vs PRUNED detections per class (ONE BAR GRAPH)

Fix included:
- C1/C2/C3 sometimes "disappear" in the last CSV row because your per-row class string omits
  classes not present/updated at that timestamp.
- We expand class maps across ALL rows and forward-fill cumulative values so the final snapshot
  contains the last-known cumulative for every class that appeared at least once.
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
CONTAINER_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]  # C1..C5
DRONE_PALETTE     = ["#0072B2", "#E69F00", "#009E73"]                       # drone1..drone3

# Fade (no stripes/dots)
ALPHA_RAW    = 0.35
ALPHA_PRUNED = 0.90


def container_label(name: str) -> str:
    """container1 -> C1, container2 -> C2, ... (fallback: original string)"""
    m = re.search(r"container\s*(\d+)", str(name), flags=re.I)
    return f"C{m.group(1)}" if m else str(name)


def class_sort_key(s: str) -> int:
    m = re.search(r"\d+", str(s))
    return int(m.group()) if m else 999


def drone_sort_key(s: str) -> int:
    m = re.search(r"drone\s*(\d+)", str(s), flags=re.I)
    return int(m.group(1)) if m else 999


def build_map(keys_in_order, palette):
    # cycles if more keys than colors
    colors = [palette[i % len(palette)] for i in range(len(keys_in_order))]
    return dict(zip(keys_in_order, colors))


def load_fusion_stats_csv():
    """Load any CSV in current folder whose filename contains 'fusion_stats' (case-insensitive)."""
    pattern = "*fusion_stats*.csv"
    files = sorted(glob.glob(pattern))

    if not files:
        files = sorted([f for f in glob.glob("*.csv") if "fusion_stats" in os.path.basename(f).lower()])

    if not files:
        raise RuntimeError(f"No CSV files found matching '{pattern}' (case-insensitive) in current directory.")

    print(f"[INFO] Found {len(files)} fusion_stats file(s):")
    for f in files:
        print("  -", f)

    dfs = []
    for f in files:
        d = pd.read_csv(f)
        d["__source_file__"] = os.path.basename(f)
        dfs.append(d)

    stats = pd.concat(dfs, ignore_index=True)

    stats["stamp_ns"] = pd.to_numeric(stats.get("stamp_ns"), errors="coerce")
    stats = stats[pd.notna(stats["stamp_ns"])].copy()
    stats.sort_values("stamp_ns", inplace=True)
    stats.reset_index(drop=True, inplace=True)
    return stats


def detect_drones(cols):
    drones = set()
    for c in cols:
        m = re.search(r"raw_total_cum_(drone\d+)$", str(c))
        if m:
            drones.add(m.group(1))
    return sorted(drones, key=drone_sort_key)


def parse_class_cum(s: str) -> dict:
    """
    Parses:
      "class: container1=12; class: container2=7"
      OR "container1=12;container2=7"
    into { "container1": 12.0, ... }
    """
    if pd.isna(s):
        return {}
    s = str(s).strip()
    if not s:
        return {}
    out = {}
    for part in s.split(";"):
        part = part.strip()
        if not part or "=" not in part:
            continue
        left, right = part.split("=", 1)
        cls = left.split(":", 1)[-1].strip()
        try:
            out[cls] = float(right)
        except ValueError:
            out[cls] = 0.0
    return out


def build_cum_df(series: pd.Series) -> pd.DataFrame:
    """
    Expand a ';'-encoded cumulative map column into a DataFrame (one column per class),
    then forward-fill so missing classes keep their last-known cumulative value.
    """
    rows = [parse_class_cum(v) for v in series]
    dfm = pd.DataFrame(rows)
    # For cumulative counters, missing class on a row should keep previous cumulative value
    dfm = dfm.ffill().fillna(0.0)
    return dfm


def main():
    stats = load_fusion_stats_csv()

    # Time axis
    if stats["stamp_ns"].isna().all():
        raise RuntimeError("stamp_ns column missing or not numeric.")
    t0 = stats["stamp_ns"].min()
    stats["t_sec"] = (stats["stamp_ns"] - t0) * 1e-9

    drones = detect_drones(stats.columns)
    if not drones:
        raise RuntimeError("No drone columns detected (expected raw_total_cum_droneX).")

    drone_color_map = build_map(drones, DRONE_PALETTE)

    # -------------------------
    # 1) Global cumulative totals
    # -------------------------
    plt.figure(figsize=(8.5, 4.8))
    plt.plot(stats["t_sec"], pd.to_numeric(stats.get("raw_total_cum"), errors="coerce"), label="raw_total_cum")
    plt.plot(stats["t_sec"], pd.to_numeric(stats.get("pruned_total_cum"), errors="coerce"), label="pruned_total_cum")
    plt.title("1) Cumulative totals over time (global)")
    plt.xlabel("Time (s))")
    plt.ylabel("Cumulative count")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()

    # -------------------------
    # 2) Global pruning efficiency
    # -------------------------
    raw_tot = pd.to_numeric(stats.get("raw_total_cum"), errors="coerce")
    pru_tot = pd.to_numeric(stats.get("pruned_total_cum"), errors="coerce")
    eff = np.where(raw_tot.to_numpy() > 0, (pru_tot / raw_tot).to_numpy(), np.nan)

    plt.figure(figsize=(8.5, 4.8))
    plt.plot(stats["t_sec"], eff, label="pruned/raw")
    plt.title("2) Pruning efficiency over time (global)")
    plt.xlabel("Time (s))")
    plt.ylabel("Ratio")
    plt.ylim(0, 1.05)
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()

    # -------------------------
    # 6) Per-drone pruning efficiency (use DRONE_PALETTE)
    # -------------------------
    plt.figure(figsize=(9.5, 5.0))
    for d in drones:
        rcol = f"raw_total_cum_{d}"
        pcol = f"pruned_total_cum_{d}"
        if rcol not in stats.columns or pcol not in stats.columns:
            continue
        r = pd.to_numeric(stats[rcol], errors="coerce")
        p = pd.to_numeric(stats[pcol], errors="coerce")
        ratio = np.where(r.to_numpy() > 0, (p / r).to_numpy(), np.nan)
        plt.plot(stats["t_sec"], ratio, label=d, color=drone_color_map[d])

    plt.title("6) Per-drone pruning efficiency over time")
    plt.xlabel("Time (s from first stamp)")
    plt.ylabel("Pruned/Raw ratio")
    plt.ylim(0, 1.05)
    plt.grid(True, alpha=0.3)
    plt.legend(title="Drone", loc="best")
    plt.tight_layout()

    # ============================================================
    # FINAL SNAPSHOT FIX:
    # Build per-drone class cumulative tables across ALL rows,
    # forward-fill missing classes, then take last row.
    # ============================================================
    raw_cum_df = {}
    pru_cum_df = {}

    for d in drones:
        rcol = f"raw_cum_{d}"
        pcol = f"pruned_cum_{d}"
        if rcol in stats.columns:
            raw_cum_df[d] = build_cum_df(stats[rcol])
        if pcol in stats.columns:
            pru_cum_df[d] = build_cum_df(stats[pcol])

    # Collect classes across entire dataset (not only the last CSV row)
    classes_orig = set()
    for d, dfm in raw_cum_df.items():
        classes_orig |= set(dfm.columns)
    for d, dfm in pru_cum_df.items():
        classes_orig |= set(dfm.columns)

    if not classes_orig:
        print("[WARN] No per-class cumulative columns found (raw_classes_cum_droneX / pruned_classes_cum_droneX).")
        plt.show()
        return

    # Map original -> display Cx
    class_disp_map = {c: container_label(c) for c in classes_orig}

    # Display classes list (C1..C5 ordered)
    classes_disp = sorted(set(class_disp_map.values()), key=class_sort_key)

    # Final maps per drone (after forward-fill)
    raw_final = {d: raw_cum_df[d].iloc[-1].to_dict() for d in raw_cum_df}
    pru_final = {d: pru_cum_df[d].iloc[-1].to_dict() for d in pru_cum_df}

    # -------------------------
    # 9) Final snapshot heatmap: per-class prune ratio per drone
    #     (pruned_class_cum / raw_class_cum)
    # -------------------------
    
    '''
    
    ratio_mat = np.full((len(classes_disp), len(drones)), np.nan, dtype=float)

    for j, d in enumerate(drones):
        raw_map_orig = raw_final.get(d, {})
        pru_map_orig = pru_final.get(d, {})

        # aggregate into Cx buckets (in case names vary)
        raw_map = {}
        pru_map = {}
        for k, v in raw_map_orig.items():
            kk = class_disp_map.get(k, k)
            raw_map[kk] = raw_map.get(kk, 0.0) + float(v)
        for k, v in pru_map_orig.items():
            kk = class_disp_map.get(k, k)
            pru_map[kk] = pru_map.get(kk, 0.0) + float(v)

        for i, cls in enumerate(classes_disp):
            rv = raw_map.get(cls, 0.0)
            pv = pru_map.get(cls, 0.0)
            ratio_mat[i, j] = (pv / rv) if rv > 0 else np.nan

    plt.figure(figsize=(max(7, 1.6 * len(drones) + 2), max(3.8, 0.9 * len(classes_disp) + 2)))
    plt.imshow(ratio_mat, aspect="auto")
    plt.title("9) Class prune ratio per drone (final snapshot)\n(pruned_class_cum / raw_class_cum)")
    plt.xlabel("Drone")
    plt.ylabel("Class")
    plt.xticks(np.arange(len(drones)), drones)
    plt.yticks(np.arange(len(classes_disp)), classes_disp)

    for i in range(len(classes_disp)):
        for j in range(len(drones)):
            v = ratio_mat[i, j]
            txt = "—" if np.isnan(v) else f"{v:.2f}"
            plt.text(j, i, txt, ha="center", va="center")

    plt.colorbar(label="Pruned/Raw ratio")
    plt.tight_layout()

    # -------------------------
    # 10) ONE BAR GRAPH: per drone per class raw vs pruned (final snapshot)
    #     - class colors use CONTAINER_PALETTE (C1..)
    #     - drone colors use DRONE_PALETTE (legend)
    #     - raw vs pruned shown by alpha only (no hatch)
    # -------------------------
    def agg_to_disp(m_orig: dict) -> dict:
        out = {}
        for k, v in m_orig.items():
            kk = class_disp_map.get(k, k)
            out[kk] = out.get(kk, 0.0) + float(v)
        return out

    raw_maps_disp = {d: agg_to_disp(raw_final.get(d, {})) for d in drones}
    pru_maps_disp = {d: agg_to_disp(pru_final.get(d, {})) for d in drones}

    raw_vals = np.array([[raw_maps_disp[d].get(cls, 0.0) for d in drones] for cls in classes_disp], dtype=float)
    pru_vals = np.array([[pru_maps_disp[d].get(cls, 0.0) for d in drones] for cls in classes_disp], dtype=float)

    n_classes = len(classes_disp)
    n_drones = len(drones)

    group_gap = 1.0
    bar_w = 0.18
    pair_gap = 0.05
    drone_gap = 0.10

    drone_slot_w = 2 * bar_w + pair_gap + drone_gap
    class_group_w = n_drones * drone_slot_w

    x0 = np.arange(n_classes) * (class_group_w + group_gap)

    # container colors in order C1..C5
    container_color_map = build_map(classes_disp, CONTAINER_PALETTE)

    plt.figure(figsize=(max(10, 1.1 * n_classes + 6), 5.8))

    for j, d in enumerate(drones):
        drone_offset = j * drone_slot_w
        x_raw = x0 + drone_offset
        x_pru = x_raw + bar_w + pair_gap

        # RAW (faded)
        plt.bar(
            x_raw, raw_vals[:, j],
            width=bar_w,
            color=[container_color_map[c] for c in classes_disp],
            alpha=ALPHA_RAW,
            edgecolor="black", linewidth=0.4
        )

        # PRUNED (stronger)
        plt.bar(
            x_pru, pru_vals[:, j],
            width=bar_w,
            color=[container_color_map[c] for c in classes_disp],
            alpha=ALPHA_PRUNED,
            edgecolor="black", linewidth=0.4
        )

    # Legends:
    drone_handles = [
        plt.Line2D([0], [0], marker="s", color="none",
                   markerfacecolor=drone_color_map[drones[i]],
                   markersize=10, label=drones[i])
        for i in range(n_drones)
    ]

    container_handles = [
        plt.Rectangle((0, 0), 1, 1, facecolor=container_color_map[c], edgecolor="black", label=c)
        for c in classes_disp
    ]

    raw_handle = plt.Rectangle((0, 0), 1, 1, facecolor="gray", edgecolor="black", alpha=ALPHA_RAW, label="raw")
    pru_handle = plt.Rectangle((0, 0), 1, 1, facecolor="gray", edgecolor="black", alpha=ALPHA_PRUNED, label="pruned")

    plt.title("10) Final snapshot: per-drone per-class Raw vs Pruned (grouped bars)")
    plt.xlabel("Class")
    plt.ylabel("Count (final cumulative)")
    plt.grid(True, axis="y", alpha=0.3)

    center = x0 + (class_group_w - drone_gap) / 2.0 - (bar_w + pair_gap) / 2.0
    plt.xticks(center, classes_disp, rotation=25, ha="right")

    ax = plt.gca()
    leg1 = ax.legend(handles=drone_handles, title="Drone", loc="upper left", bbox_to_anchor=(1.01, 1.0))
    ax.add_artist(leg1)

    leg2 = ax.legend(handles=[raw_handle, pru_handle], title="Type (opacity)", loc="upper left", bbox_to_anchor=(1.01, 0.70))
    ax.add_artist(leg2)

    ax.legend(handles=container_handles, title="Container class", loc="upper left", bbox_to_anchor=(1.01, 0.40))

    plt.tight_layout()
    '''
    plt.show()
   

if __name__ == "__main__":
    main()
