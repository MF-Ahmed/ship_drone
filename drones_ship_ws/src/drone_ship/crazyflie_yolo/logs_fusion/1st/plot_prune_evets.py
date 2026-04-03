#!/usr/bin/env python3
"""
Plots from prune_events*.csv (auto-detected in current directory):
  A) Global cumulative prunes over time
  B) Per-drone cumulative prunes over time
  C) Prune counts per drone per class (grouped bars)
  D) Prune reasons per drone (time vs mahal) (grouped bars)
  E) Distributions: age_supported_s + best_d2 (for mahal-pruned)

Run:
  python3 plot_prune_events.py
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def load_prune_events_csv() -> pd.DataFrame:
    """
    Load any CSV in current folder whose filename contains 'prune_events' (case-insensitive).
    If multiple files match, concatenate them by time order (stamp_ns).
    """
    pattern = "*prune_events*.csv"
    files = sorted(glob.glob(pattern))

    # fallback: case-insensitive scan of all csv
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

    # Ensure stamp_ns exists and is numeric, then sort so time series plots work correctly.
    df["stamp_ns"] = pd.to_numeric(df.get("stamp_ns"), errors="coerce")
    df = df[pd.notna(df["stamp_ns"])].copy()
    df.sort_values("stamp_ns", inplace=True)
    df.reset_index(drop=True, inplace=True)
    return df


def explode_drones_seen(df: pd.DataFrame) -> pd.DataFrame:
    """
    Expand rows where drones_seen may contain multiple drones like:
      "drone1,drone2" or "drone1 drone2"
    into one row per drone.
    """
    d = df.copy()
    d["drones_seen"] = d["drones_seen"].fillna("").astype(str)

    # Normalize separators to commas then split
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


def main():
    df = load_prune_events_csv()

    # Types
    df["stamp_ns"] = pd.to_numeric(df.get("stamp_ns"), errors="coerce")
    if df["stamp_ns"].isna().all():
        raise RuntimeError("stamp_ns missing or not numeric.")

    df["track_id"] = pd.to_numeric(df.get("track_id"), errors="coerce")
    df["class_name"] = df.get("class_name", "").astype(str)

    # Time axis: seconds from first stamp
    t0 = df["stamp_ns"].min()
    df["t_sec"] = (df["stamp_ns"] - t0) * 1e-9

    # Explode by drone (drones_seen)
    dfx = explode_drones_seen(df)

    drones = sorted(dfx["drone"].unique().tolist())
    classes = sorted(dfx["class_name"].dropna().unique().tolist())

    # =========================
    # A + B in ONE figure (single axes, no subplots)
    # =========================
    plt.figure(figsize=(9.5, 5.2))

    # --- Global cumulative prunes ---
    g = df.sort_values("t_sec")
    cum_global = np.arange(1, len(g) + 1)
    plt.plot(
        g["t_sec"].to_numpy(),
        cum_global,
        linewidth=2.6,
        label="Global (all drones)"
    )

    # --- Per-drone cumulative prunes ---
    for dr in drones:
        sub = dfx[dfx["drone"] == dr].sort_values("t_sec")
        if sub.empty:
            continue
        cum_d = np.arange(1, len(sub) + 1)
        plt.plot(
            sub["t_sec"].to_numpy(),
            cum_d,
            linewidth=1.6,
            alpha=0.9,
            label=dr
        )

    plt.title("Cumulative pruned tracks over time")
    plt.xlabel("Time (s from first stamp)")
    plt.ylabel("Cumulative count")
    plt.grid(True, alpha=0.3)
    plt.legend(title="Curve", loc="best")
    plt.tight_layout()


    # =========================
    # C) Prune counts per drone per class (grouped bars)
    # =========================
    counts_dc = (
        dfx.groupby(["drone", "class_name"])
        .size()
        .unstack(fill_value=0)
        .reindex(index=drones, columns=classes, fill_value=0)
    )

    x = np.arange(len(drones))
    n_classes = max(len(classes), 1)
    width = 0.85 / n_classes

    plt.figure(figsize=(10, 5.2))
    for i, cls in enumerate(classes):
        plt.bar(x + (i - (n_classes - 1) / 2) * width, counts_dc[cls].values, width=width, label=cls)

    plt.title("C) Prune event counts per drone per class")
    plt.xlabel("Drone")
    plt.ylabel("Count (events)")
    plt.xticks(x, drones)
    plt.grid(True, axis="y", alpha=0.3)
    plt.legend(title="class_name", loc="best")
    plt.tight_layout()

    # =========================
    # D) Prune reasons per drone (time vs mahal) (grouped bars)
    # =========================
    #dfx["reason_time"] = pd.to_numeric(dfx.get("reason_time", 0), errors="coerce").fillna(0).astype(int)
    #dfx["reason_mahal"] = pd.to_numeric(dfx.get("reason_mahal", 0), errors="coerce").fillna(0).astype(int)

    #reasons = dfx.groupby("drone")[["reason_time", "reason_mahal"]].sum().reindex(drones).fillna(0)

    #plt.figure(figsize=(9.5, 5.2))
    #w = 0.35
    #plt.bar(x - w / 2, reasons["reason_time"].values, width=w, label="reason_time")
    #plt.bar(x + w / 2, reasons["reason_mahal"].values, width=w, label="reason_mahal")
    #plt.title("D) Prune reasons per drone")
    #plt.xlabel("Drone")
    #plt.ylabel("Count (events)")
    #plt.xticks(x, drones)
    #plt.grid(True, axis="y", alpha=0.3)
    #plt.legend(loc="best")
    #plt.tight_layout()

    # =========================
    # E) Distributions: age_supported_s + best_d2 (for mahal-pruned)
    # =========================
    #df["age_supported_s"] = pd.to_numeric(df.get("age_supported_s"), errors="coerce")
    #df["best_d2"] = pd.to_numeric(df.get("best_d2"), errors="coerce")
    #df["reason_mahal"] = pd.to_numeric(df.get("reason_mahal", 0), errors="coerce").fillna(0).astype(int)

    #plt.figure(figsize=(8.5, 4.8))
    #age = df["age_supported_s"].dropna().to_numpy()
    #plt.hist(age, bins=25)
    #plt.title("E1) Distribution of age_supported_s at prune")
    #plt.xlabel("age_supported_s (s)")
    #plt.ylabel("Frequency")
   # plt.grid(True, axis="y", alpha=0.3)
    #plt.tight_layout()

    #plt.figure(figsize=(8.5, 4.8))
    #best = df.loc[df["reason_mahal"] == 1, "best_d2"].dropna().to_numpy()
    #best = best[best >= 0]  # filter "-1" etc
    #plt.hist(best, bins=25)
    #plt.title("E2) Distribution of best_d2 for mahal-pruned events")
    #plt.xlabel("best_d2")
    #plt.ylabel("Frequency")
    #plt.grid(True, axis="y", alpha=0.3)
    #plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()
