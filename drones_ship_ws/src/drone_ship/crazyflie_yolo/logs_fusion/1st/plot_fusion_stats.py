#!/usr/bin/env python3
"""
Plots from fusion_stats*.csv (auto-detected in current directory):
  1) Global cumulative totals (raw vs pruned) over time
  2) Global pruning efficiency (pruned/raw) over time
  6) Per-drone pruning efficiency (pruned/raw) over time
  9) Final snapshot heatmap: per-class prune ratio per drone (pruned_class_cum/raw_class_cum)
  10) Final snapshot: per-drone RAW vs PRUNED detections per class (ONE BAR GRAPH)

Just run:
  python3 plot_fusion_stats.py
"""

import os
import glob
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_fusion_stats_csv():
    """
    Load any CSV in current folder whose filename contains 'fusion_stats' (case-insensitive).
    If multiple files match, concatenate them by time order (stamp_ns).
    """
    pattern = "*fusion_stats*.csv"
    files = sorted(glob.glob(pattern))

    # fallback: case-insensitive scan of all csv
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

    # Ensure stamp_ns exists and is numeric, then sort so "last row" is truly latest.
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
    return sorted(drones)


def parse_class_cum(s: str) -> dict:
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

    # -------------------------
    # 1) Global cumulative totals
    # -------------------------
    plt.figure(figsize=(8.5, 4.8))
    plt.plot(stats["t_sec"], pd.to_numeric(stats.get("raw_total_cum"), errors="coerce"), label="raw_total_cum")
    plt.plot(stats["t_sec"], pd.to_numeric(stats.get("pruned_total_cum"), errors="coerce"), label="pruned_total_cum")
    plt.title("1) Cumulative totals over time (global)")
    plt.xlabel("Time (s from first stamp)")
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
    plt.xlabel("Time (s from first stamp)")
    plt.ylabel("Ratio")
    plt.ylim(0, 1.05)
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()

    # -------------------------
    # 6) Per-drone pruning efficiency
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
        plt.plot(stats["t_sec"], ratio, label=d)

    plt.title("6) Per-drone pruning efficiency over time")
    plt.xlabel("Time (s from first stamp)")
    plt.ylabel("Pruned/Raw ratio")
    plt.ylim(0, 1.05)
    plt.grid(True, alpha=0.3)
    plt.legend(title="Drone", loc="best")
    plt.tight_layout()

    # -------------------------
    # 9) Final snapshot heatmap: per-class prune ratio per drone
    # -------------------------
    raw_cls_cols = {d: f"raw_classes_cum_{d}" for d in drones if f"raw_classes_cum_{d}" in stats.columns}
    pru_cls_cols = {d: f"pruned_classes_cum_{d}" for d in drones if f"pruned_classes_cum_{d}" in stats.columns}

    last = stats.iloc[-1]

    classes = set()
    for d, col in raw_cls_cols.items():
        classes |= set(parse_class_cum(last[col]).keys())
    for d, col in pru_cls_cols.items():
        classes |= set(parse_class_cum(last[col]).keys())
    classes = sorted(classes)

    ratio_mat = np.full((len(classes), len(drones)), np.nan, dtype=float)

    for j, d in enumerate(drones):
        raw_map = parse_class_cum(last.get(raw_cls_cols.get(d, ""), np.nan))
        pru_map = parse_class_cum(last.get(pru_cls_cols.get(d, ""), np.nan))
        for i, cls in enumerate(classes):
            rv = raw_map.get(cls, 0.0)
            pv = pru_map.get(cls, 0.0)
            ratio_mat[i, j] = (pv / rv) if rv > 0 else np.nan

    plt.figure(figsize=(max(7, 1.6 * len(drones) + 2), max(3.8, 0.9 * len(classes) + 2)))
    plt.imshow(ratio_mat, aspect="auto")
    plt.title("9) Class prune ratio per drone (final snapshot)\n(pruned_class_cum / raw_class_cum)")
    plt.xlabel("Drone")
    plt.ylabel("Class (class_name)")
    plt.xticks(np.arange(len(drones)), drones)
    plt.yticks(np.arange(len(classes)), classes)

    for i in range(len(classes)):
        for j in range(len(drones)):
            v = ratio_mat[i, j]
            txt = "—" if np.isnan(v) else f"{v:.2f}"
            plt.text(j, i, txt, ha="center", va="center")

    plt.colorbar(label="Pruned/Raw ratio")
    plt.tight_layout()

    # -------------------------
    # 10) ONE BAR GRAPH: per drone per class raw vs pruned (final snapshot)
    # -------------------------
    if classes:
        raw_maps = {d: parse_class_cum(last.get(f"raw_classes_cum_{d}", np.nan)) for d in drones}
        pru_maps = {d: parse_class_cum(last.get(f"pruned_classes_cum_{d}", np.nan)) for d in drones}

        raw_vals = np.array([[raw_maps[d].get(cls, 0.0) for d in drones] for cls in classes], dtype=float)
        pru_vals = np.array([[pru_maps[d].get(cls, 0.0) for d in drones] for cls in classes], dtype=float)

        n_classes = len(classes)
        n_drones = len(drones)

        group_gap = 1.0
        bar_w = 0.18
        pair_gap = 0.05
        drone_gap = 0.10

        drone_slot_w = 2 * bar_w + pair_gap + drone_gap
        class_group_w = n_drones * drone_slot_w

        x0 = np.arange(n_classes) * (class_group_w + group_gap)

        drone_colors = [
            "tab:blue", "tab:orange", "tab:green", "tab:red",
            "tab:purple", "tab:brown", "tab:pink", "tab:gray",
            "tab:olive", "tab:cyan",
        ]

        plt.figure(figsize=(max(10, 1.1 * n_classes + 6), 5.8))

        for j, d in enumerate(drones):
            c = drone_colors[j % len(drone_colors)]
            drone_offset = j * drone_slot_w
            x_raw = x0 + drone_offset
            x_pru = x_raw + bar_w + pair_gap

            plt.bar(
                x_raw, raw_vals[:, j],
                width=bar_w,
                color=c, alpha=0.35,
                edgecolor="black", linewidth=0.4,
                hatch="///"
            )

            plt.bar(
                x_pru, pru_vals[:, j],
                width=bar_w,
                color=c, alpha=0.90,
                edgecolor="black", linewidth=0.4
            )

        drone_handles = [
            plt.Line2D([0], [0], marker="s", color="none",
                       markerfacecolor=drone_colors[i % len(drone_colors)],
                       markersize=10, label=drones[i])
            for i in range(n_drones)
        ]
        raw_handle = plt.Rectangle((0, 0), 1, 1, facecolor="lightgray", edgecolor="black", hatch="///", label="raw")
        pru_handle = plt.Rectangle((0, 0), 1, 1, facecolor="gray", edgecolor="black", label="pruned")

        plt.title("10) Final snapshot: per-drone per-class Raw vs Pruned (grouped bars)")
        plt.xlabel("Class (class_name)")
        plt.ylabel("Count (final cumulative)")
        plt.grid(True, axis="y", alpha=0.3)

        center = x0 + (class_group_w - drone_gap) / 2.0 - (bar_w + pair_gap) / 2.0
        plt.xticks(center, classes, rotation=25, ha="right")

        leg1 = plt.legend(handles=drone_handles, title="Drone", loc="upper left", bbox_to_anchor=(1.01, 1.0))
        plt.gca().add_artist(leg1)
        plt.legend(handles=[raw_handle, pru_handle], title="Type", loc="upper left", bbox_to_anchor=(1.01, 0.55))

        plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
