#!/usr/bin/env python3
import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

DEFAULT_FILES = [
    "drone1_tracking_metrics.csv",
    "drone2_tracking_metrics.csv",
    "drone3_tracking_metrics.csv",
]

REQUIRED_BASE = ["method", "IDF1", "MOTA", "MedErr", "RMSE", "P95"]
OLD_COUNTERS = ["FP", "FN", "IDSW", "Frag", "GT"]


def _read_csv_safely(path: Path) -> pd.DataFrame:
    try:
        return pd.read_csv(path)
    except Exception as e1:
        print(f"[WARN] Standard read failed for {path.name}: {e1}")
        return pd.read_csv(path, engine="python", on_bad_lines="skip")


def _normalize_columns(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    df.columns = [str(c).strip() for c in df.columns]

    # Map possible "cum" columns -> classic names
    rename_map = {}
    for a, b in [("FP_cum", "FP"), ("FN_cum", "FN"), ("IDSW_cum", "IDSW"),
                 ("Frag_cum", "Frag"), ("GT_cum", "GT")]:
        if b not in df.columns and a in df.columns:
            rename_map[a] = b
    if rename_map:
        df = df.rename(columns=rename_map)

    if "stamp_sec" not in df.columns and "stamp" in df.columns:
        df = df.rename(columns={"stamp": "stamp_sec"})

    # numeric conversion
    numeric_cols = ["IDF1", "MOTA", "MedErr", "RMSE", "P95",
                    "FP", "FN", "IDSW", "Frag", "GT", "stamp_sec"]
    for c in numeric_cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")

    # normalize method
    if "method" in df.columns:
        df["method"] = df["method"].astype(str).str.strip().str.lower()
        df["method"] = df["method"].replace({
            "our": "ours",
            "ekf": "ours",
            "ekf_tracker": "ours",
            "ab3d": "ab3dmot",
            "ab3dmot_3d": "ab3dmot",
            "sort_tracker": "sort",
        })

    return df


def _required_missing(df: pd.DataFrame):
    req = REQUIRED_BASE + OLD_COUNTERS
    return [c for c in req if c not in df.columns]


def load_only_these_files(files):
    cwd = Path.cwd()
    paths = [cwd / f for f in files]

    print("[INFO] Reading ONLY these files (current directory):")
    for p in paths:
        print(f"  - {p}")

    frames = []
    for p in paths:
        if not p.exists():
            print(f"[WARN] File not found: {p.name}")
            continue
        if p.stat().st_size == 0:
            print(f"[WARN] Empty file skipped: {p.name}")
            continue

        try:
            df = _read_csv_safely(p)
            if df.empty:
                print(f"[WARN] Empty dataframe skipped: {p.name}")
                continue

            df = _normalize_columns(df)
            missing = _required_missing(df)
            if missing:
                print(f"[WARN] Skipping {p.name} (missing columns: {missing})")
                print(f"       Available columns: {list(df.columns)}")
                continue

            df["__source_file"] = p.name
            stem = p.stem.lower()
            if "drone1" in stem:
                df["drone"] = "drone1"
            elif "drone2" in stem:
                df["drone"] = "drone2"
            elif "drone3" in stem:
                df["drone"] = "drone3"
            else:
                df["drone"] = "unknown"

            frames.append(df)
        except Exception as e:
            print(f"[WARN] Failed to read {p.name}: {e}")

    if not frames:
        raise RuntimeError("No valid metrics CSVs loaded from the provided files.")

    return pd.concat(frames, ignore_index=True)


def summarize_final(df: pd.DataFrame):
    keep_methods = ["ours", "ab3dmot", "sort"]
    d = df[df["method"].isin(keep_methods)].copy()
    if d.empty:
        d = df.copy()

    # Avoid pandas groupby.apply deprecation:
    # take the last row by stamp_sec per (drone, method)
    if "stamp_sec" in d.columns and d["stamp_sec"].notna().any():
        d = d.sort_values("stamp_sec")
    finals = d.groupby(["drone", "method"], as_index=False).tail(1).reset_index(drop=True)

    # aggregated rows
    rows = []
    for method, gm in finals.groupby("method"):
        FP = gm["FP"].sum(skipna=True)
        FN = gm["FN"].sum(skipna=True)
        IDSW = gm["IDSW"].sum(skipna=True)
        GT = gm["GT"].sum(skipna=True)

        mota_global = np.nan
        if GT > 0:
            mota_global = 1.0 - (FN + FP + IDSW) / GT

        row = {
            "Method": method,
            "Drones": len(gm),
            "IDF1_mean": gm["IDF1"].mean(skipna=True),
            "MOTA_global": mota_global,
            "MedErr_mean": gm["MedErr"].mean(skipna=True),
            "RMSE_mean": gm["RMSE"].mean(skipna=True),
            "P95_mean": gm["P95"].mean(skipna=True),
            "IDSW_sum": IDSW,
            "Frag_sum": gm["Frag"].sum(skipna=True),
            "FP_sum": FP,
            "FN_sum": FN,
            "GT_sum": GT,
        }
        rows.append(row)

    summary = pd.DataFrame(rows)

    order = {"ours": 0, "ab3dmot": 1, "sort": 2}
    if not summary.empty:
        summary["__ord"] = summary["Method"].map(order).fillna(999)
        summary = summary.sort_values("__ord").drop(columns="__ord")

    finals_order = {"ours": 0, "ab3dmot": 1, "sort": 2}
    finals["__ord"] = finals["method"].map(finals_order).fillna(999)
    finals = finals.sort_values(["drone", "__ord"]).drop(columns="__ord")

    return summary, finals


def _format_table_for_display(df: pd.DataFrame) -> pd.DataFrame:
    out = df.copy()
    for c in out.columns:
        if out[c].dtype.kind in "fc":  # float
            out[c] = out[c].map(lambda x: "nan" if not np.isfinite(x) else f"{x:.3f}")
        elif out[c].dtype.kind in "iu":  # int
            out[c] = out[c].astype(int).astype(str)
    return out


def save_table_as_image(df: pd.DataFrame, out_path: Path, title: str):
    df_disp = _format_table_for_display(df)

    fig_w = max(8, 0.9 * len(df_disp.columns) + 2)
    fig_h = max(2.5, 0.45 * (len(df_disp) + 2))

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.axis("off")

    table = ax.table(
        cellText=df_disp.values,
        colLabels=df_disp.columns.tolist(),
        cellLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 1.3)

    ax.set_title(title, fontsize=14, pad=16)

    fig.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] Saved table image: {out_path.resolve()}")


def main():
    parser = argparse.ArgumentParser(
        description="Summarize tracking metrics from ONLY drone1/2/3 CSV files in current directory "
                    "and save tables as CSV + JPG figures."
    )
    parser.add_argument("--files", nargs="*", default=DEFAULT_FILES,
                        help="CSV filenames in current directory (default: drone1/2/3_tracking_metrics.csv)")
    parser.add_argument("--save-summary", default="tracking_metrics_summary.csv",
                        help="Save aggregated summary CSV")
    parser.add_argument("--save-finals", default="tracking_metrics_final_rows.csv",
                        help="Save final rows per drone/method CSV")
    parser.add_argument("--fig-summary", default="table_summary.jpg",
                        help="Output JPG for summary table")
    parser.add_argument("--fig-finals", default="table_final_rows.jpg",
                        help="Output JPG for per-drone final rows table")
    args = parser.parse_args()

    df = load_only_these_files(args.files)
    summary, finals = summarize_final(df)

    finals_cols = ["drone", "method", "IDF1", "MOTA", "MedErr", "RMSE", "P95",
                   "IDSW", "Frag", "FP", "FN", "GT"]
    finals_small = finals[[c for c in finals_cols if c in finals.columns]].copy()

    pd.set_option("display.width", 200)
    pd.set_option("display.max_columns", 80)

    print("\n=== Final row per drone × method ===")
    print(finals_small.to_string(index=False))

    print("\n=== Aggregated over drones (global MOTA from summed counts) ===")
    print(summary.to_string(index=False))

    # Save CSVs (fixed)
    summary.to_csv(args.save_summary, index=False)
    finals_small.to_csv(args.save_finals, index=False)

    print(f"\n[INFO] Saved summary CSV: {Path(args.save_summary).resolve()}")
    print(f"[INFO] Saved finals CSV:  {Path(args.save_finals).resolve()}")

    # Save JPG tables
    save_table_as_image(summary, Path(args.fig_summary),
                        "Tracking Metrics Summary (Aggregated over Drones)")
    save_table_as_image(finals_small, Path(args.fig_finals),
                        "Final Cumulative Metrics per Drone × Method")


if __name__ == "__main__":
    main()