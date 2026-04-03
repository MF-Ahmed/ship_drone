#!/usr/bin/env python3
from pathlib import Path
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def save_table_image(df, out_path, title):
    disp = df.copy()
    for c in disp.columns:
        if pd.api.types.is_numeric_dtype(disp[c]):
            disp[c] = disp[c].map(lambda x: "nan" if pd.isna(x) else f"{x:.3f}")

    fig_w = max(7, 1.15 * len(disp.columns) + 1)
    fig_h = max(2.5, 0.55 * (len(disp) + 2))

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.axis("off")
    ax.set_title(title, fontsize=14, pad=12)

    t = ax.table(
        cellText=disp.values,
        colLabels=disp.columns.tolist(),
        cellLoc="center",
        loc="center"
    )
    t.auto_set_font_size(False)
    t.set_fontsize(10)
    t.scale(1.0, 1.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"[INFO] saved JPG: {Path(out_path).resolve()}")


def to_ieee_latex_table(df, out_tex_path, caption, label):
    """
    Create a compact IEEE-style LaTeX table (single-column friendly).
    """
    # Build LaTeX manually for maximum control
    cols = df.columns.tolist()
    lines = []
    lines.append("\\begin{table}[t]")
    lines.append("\\centering")
    lines.append(f"\\caption{{{caption}}}")
    lines.append(f"\\label{{{label}}}")
    lines.append("\\setlength{\\tabcolsep}{3.5pt}")
    lines.append("\\renewcommand{\\arraystretch}{1.05}")
    lines.append("\\begin{tabular}{lcccc}")  # Method + 4 numeric columns
    lines.append("\\toprule")
    lines.append(" & ".join([f"\\textbf{{{c}}}" for c in cols]) + " \\\\")
    lines.append("\\midrule")

    for _, row in df.iterrows():
        r = []
        for c in cols:
            v = row[c]
            if isinstance(v, (float, np.floating)) and np.isfinite(v):
                r.append(f"{v:.3f}")
            elif isinstance(v, (int, np.integer)):
                r.append(str(int(v)))
            else:
                r.append(str(v))
        lines.append(" & ".join(r) + " \\\\")

    lines.append("\\bottomrule")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}")

    out_tex_path = Path(out_tex_path)
    out_tex_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"[INFO] saved LaTeX: {out_tex_path.resolve()}")


def main():
    ap = argparse.ArgumentParser(description="Summarize fusion metrics (CI vs FTCI) from fusion_metrics.csv in current directory.")
    ap.add_argument("--csv", default="fusion_metrics.csv", help="Input CSV (current directory)")
    ap.add_argument("--out-csv", default="fusion_summary.csv", help="Output summary CSV")
    ap.add_argument("--out-jpg", default="fusion_table.jpg", help="Output JPG table")
    ap.add_argument("--out-tex", default="fusion_table.tex", help="Output LaTeX table snippet")
    args = ap.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path.resolve()}")

    df = pd.read_csv(csv_path)
    if df.empty:
        raise RuntimeError(f"CSV is empty: {csv_path.resolve()}")

    df.columns = [c.strip() for c in df.columns]
    df["method"] = df["method"].astype(str).str.lower().str.strip()
    df = df.sort_values("stamp_sec")

    # Last row per method = final cumulative
    last = df.groupby("method", as_index=False).tail(1).reset_index(drop=True)

    # Backward-compatible: keep NEES only if present
    has_nees = "MeanNEES" in last.columns

    # Build output table columns
    keep = ["method", "MedErr", "RMSE", "P95", "MeanLogDet", "N"]
    if has_nees:
        keep.insert(4, "MeanNEES")  # insert before MeanLogDet

    missing = [c for c in keep if c not in last.columns]
    if missing:
        raise RuntimeError(f"Missing columns in CSV: {missing}\nAvailable: {list(last.columns)}")

    out = last[keep].copy()

    # Rename for display/paper
    rename_map = {
        "method": "Method",
        "MedErr": "MedErr (m)↓",
        "RMSE": "RMSE (m)↓",
        "P95": "P95 (m)↓",
        "MeanLogDet": "Mean logdet(P)↓",
        "N": "Matched N",
    }
    if has_nees:
        rename_map["MeanNEES"] = "Mean NEES↓"

    out = out.rename(columns=rename_map)

    # Order rows
    order = {"ci": 0, "ftci": 1}
    out["__ord"] = out["Method"].map(order).fillna(999)
    out = out.sort_values("__ord").drop(columns="__ord")

    print("\n=== Fusion Summary (final cumulative) ===")
    print(out.to_string(index=False))

    # Save CSV + JPG
    out.to_csv(args.out_csv, index=False)
    print(f"\n[INFO] saved CSV: {Path(args.out_csv).resolve()}")

    save_table_image(out, args.out_jpg, "Fusion Comparison: CI vs FTCI")

    # Save compact IEEE LaTeX (prefer: Method, MedErr, RMSE, P95, Mean logdet(P))
    # If NEES exists, it will appear too — you can delete that column if you want a 5-col table.
    to_ieee_latex_table(
        out.drop(columns=["Matched N"], errors="ignore"),  # usually exclude N from paper table
        args.out_tex,
        caption="Fusion performance comparison between CI and fault-tolerant CI (FTCI). Lower is better ($\\downarrow$).",
        label="tab:fusion_compact"
    )


if __name__ == "__main__":
    main()