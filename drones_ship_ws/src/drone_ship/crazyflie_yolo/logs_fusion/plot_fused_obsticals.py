#!/usr/bin/env python3
"""
Paper-ready plots from a fused_obstacles CSV:
  1) Active fused tracks over time
  2) Uncertainty over time (logdet(P))

Input file (default): Path1_fused_obstacles_*.csv in current directory
or provide explicitly:
  python3 plot_fused_obstacles_metrics.py --csv Path1_fused_obstacles_2026-01-09_15-46-01.csv

Outputs:
- Shows plots interactively
- Optional save to PDF/PNG:
  python3 plot_fused_obstacles_metrics.py --save --outdir figs --prefix path1

Notes:
- logdet(P) computed from cov00..cov22 (3x3) per row.
- Active tracks computed by time-binning, then counting unique fused_id per bin.
"""

import os
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def find_default_csv():
    patt = "Path*_fused_obstacles*.csv"
    files = sorted(glob.glob(patt))
    if not files:
        # fallback: any fused_obstacles*.csv
        files = sorted([f for f in glob.glob("*.csv") if "fused_obstacles" in os.path.basename(f).lower()])
    if not files:
        raise RuntimeError("No fused_obstacles CSV found. Use --csv to specify a file.")
    return files[-1]  # newest by name (often timestamped)


def maybe_save(fig, outdir, name, save_pdf=True, save_png=True, dpi=300):
    if outdir is None:
        return
    os.makedirs(outdir, exist_ok=True)
    if save_pdf:
        fig.savefig(os.path.join(outdir, f"{name}.pdf"), bbox_inches="tight")
    if save_png:
        fig.savefig(os.path.join(outdir, f"{name}.png"), dpi=dpi, bbox_inches="tight")


def compute_logdetP(df: pd.DataFrame) -> pd.Series:
    """
    Compute logdet(P) for 3x3 covariance per row.
    Uses slogdet; returns NaN if covariance not positive definite.
    """
    needed = ["cov00","cov01","cov02","cov10","cov11","cov12","cov20","cov21","cov22"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise RuntimeError(f"Missing covariance columns: {missing}")

    logdets = np.full(len(df), np.nan, dtype=float)

    c00 = df["cov00"].to_numpy(float)
    c01 = df["cov01"].to_numpy(float)
    c02 = df["cov02"].to_numpy(float)
    c10 = df["cov10"].to_numpy(float)
    c11 = df["cov11"].to_numpy(float)
    c12 = df["cov12"].to_numpy(float)
    c20 = df["cov20"].to_numpy(float)
    c21 = df["cov21"].to_numpy(float)
    c22 = df["cov22"].to_numpy(float)

    for i in range(len(df)):
        P = np.array([
            [c00[i], c01[i], c02[i]],
            [c10[i], c11[i], c12[i]],
            [c20[i], c21[i], c22[i]],
        ], dtype=float)

        # symmetrize (defensive against tiny asymmetry)
        P = 0.5 * (P + P.T)

        sign, ld = np.linalg.slogdet(P)
        if sign > 0 and np.isfinite(ld):
            logdets[i] = ld
        else:
            logdets[i] = np.nan

    return pd.Series(logdets, index=df.index, name="logdetP")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=str, default=None, help="Path to fused_obstacles CSV")
    ap.add_argument("--bin-s", type=float, default=0.5, help="Time bin size in seconds for active track count")
    ap.add_argument("--smooth-s", type=float, default=1.5, help="Rolling smoothing window (seconds) for mean logdet(P)")
    ap.add_argument("--save", action="store_true", help="Save figures to disk (PDF+PNG)")
    ap.add_argument("--outdir", type=str, default="figs", help="Output directory")
    ap.add_argument("--prefix", type=str, default="fused", help="Filename prefix for saved figures")
    args = ap.parse_args()

    csv_path = args.csv or find_default_csv()
    print(f"[INFO] Using CSV: {csv_path}")

    df = pd.read_csv(csv_path)

    # Required columns
    for col in ["stamp_ns", "fused_id"]:
        if col not in df.columns:
            raise RuntimeError(f"Missing required column '{col}' in CSV.")

    df["stamp_ns"] = pd.to_numeric(df["stamp_ns"], errors="coerce")
    df = df[pd.notna(df["stamp_ns"])].copy()
    df.sort_values("stamp_ns", inplace=True)
    df.reset_index(drop=True, inplace=True)

    # Time axis (seconds from first stamp)
    t0 = df["stamp_ns"].min()
    df["t_sec"] = (df["stamp_ns"] - t0) * 1e-9

    # Compute logdet(P)
    df["logdetP"] = compute_logdetP(df)

    # ----------------------------
    # Plot 1: Active tracks over time
    # ----------------------------
    bin_s = float(args.bin_s)
    df["t_bin"] = np.floor(df["t_sec"] / bin_s).astype(int)

    active = df.groupby("t_bin")["fused_id"].nunique().sort_index()
    t_active = active.index.to_numpy() * bin_s

    fig1 = plt.figure(figsize=(8.6, 4.6))
    plt.plot(t_active, active.to_numpy())
    plt.title("Active fused tracks over time")
    plt.xlabel("Time (s)")
    plt.ylabel("# active fused tracks")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    # ----------------------------
    # Plot 2: Uncertainty over time (logdet(P))
    # We plot mean logdetP across active tracks per time bin (paper-friendly).
    # (Also overlays a smoothed curve)
    # ----------------------------
    # Per-bin mean over all rows in that bin; to be closer to "per-track",
    # you can instead first average per (bin, fused_id) then average across fused_id.
    per_track_bin = (
        df.groupby(["t_bin", "fused_id"])["logdetP"]
          .mean()
          .reset_index()
    )
    mean_logdet = per_track_bin.groupby("t_bin")["logdetP"].mean().sort_index()

    t_ld = mean_logdet.index.to_numpy() * bin_s
    y_ld = mean_logdet.to_numpy()

    # smoothing window in bins
    smooth_bins = max(1, int(round(args.smooth_s / bin_s)))
    y_ld_smooth = pd.Series(y_ld).rolling(smooth_bins, min_periods=1).mean().to_numpy()

    fig2 = plt.figure(figsize=(8.6, 4.6))
    plt.plot(t_ld, y_ld, label="mean logdet(P) (binned)")
    plt.plot(t_ld, y_ld_smooth, label=f"smoothed (≈{args.smooth_s:.1f}s)")
    plt.title("Fused-track uncertainty over time (log det covariance)")
    plt.xlabel("Time (s)")
    plt.ylabel("mean log det(P)")
    plt.grid(True, alpha=0.3)
    plt.legend(loc="best")
    plt.tight_layout()

    # Save if requested
    if args.save:
        outdir = args.outdir
        pref = args.prefix
        maybe_save(fig1, outdir, f"{pref}_active_tracks")
        maybe_save(fig2, outdir, f"{pref}_logdetP_over_time")
        print(f"[INFO] Saved figures to: {os.path.abspath(outdir)}")

    plt.show()


if __name__ == "__main__":
    main()
