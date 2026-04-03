#!/usr/bin/env python3
"""
Merge (append) two CSV files: output = csv1 rows + csv2 rows.

- Prompts for 2 input CSV files
- Appends contents of the 2nd to the 1st (row-wise)
- Saves into a new CSV file

Notes:
- If columns differ, it will take the union of columns and fill missing cells with empty values.
- If you want strict "must match columns", set STRICT_COLUMNS=True.
"""

import os
import sys
import pandas as pd

STRICT_COLUMNS = False  # set True to require identical columns


def pick_file(prompt: str) -> str:
    """
    Try GUI file picker; fallback to terminal input.
    """
    # Try tkinter GUI
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        path = filedialog.askopenfilename(
            title=prompt,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        root.destroy()
        if path:
            return path
    except Exception:
        pass

    # Fallback to CLI
    while True:
        path = input(f"{prompt}\nEnter path: ").strip().strip('"').strip("'")
        if os.path.isfile(path):
            return path
        print(f"❌ File not found: {path}\nTry again.\n")


def ask_output_path(default_name="merged.csv") -> str:
    """
    Ask where to save output. GUI save dialog if possible, else CLI.
    """
    # Try tkinter GUI
    try:
        import tkinter as tk
        from tkinter import filedialog

        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        path = filedialog.asksaveasfilename(
            title="Save merged CSV as...",
            defaultextension=".csv",
            initialfile=default_name,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        root.destroy()
        if path:
            return path
    except Exception:
        pass

    # Fallback to CLI
    out = input(f"Output CSV filename (default: {default_name}): ").strip()
    if not out:
        out = default_name
    return out


def main():
    print("=== CSV Appender (rows) ===")
    file1 = pick_file("Select / enter CSV file #1 (base)")
    file2 = pick_file("Select / enter CSV file #2 (to append)")

    print(f"\nReading:\n  1) {file1}\n  2) {file2}\n")

    try:
        df1 = pd.read_csv(file1)
        df2 = pd.read_csv(file2)
    except Exception as e:
        print(f"❌ Failed to read CSV: {e}")
        sys.exit(1)

    if STRICT_COLUMNS:
        if list(df1.columns) != list(df2.columns):
            print("❌ Column mismatch (STRICT_COLUMNS=True).")
            print("CSV #1 columns:", list(df1.columns))
            print("CSV #2 columns:", list(df2.columns))
            sys.exit(1)

    # Union of columns (if different)
    merged = pd.concat([df1, df2], ignore_index=True, sort=False)

    default_name = "merged.csv"
    out_path = ask_output_path(default_name=default_name)

    try:
        merged.to_csv(out_path, index=False)
    except Exception as e:
        print(f"❌ Failed to write output CSV: {e}")
        sys.exit(1)

    print("\n✅ Done.")
    print(f"Rows in CSV #1: {len(df1)}")
    print(f"Rows in CSV #2: {len(df2)}")
    print(f"Rows in merged: {len(merged)}")
    print(f"Saved to: {os.path.abspath(out_path)}")


if __name__ == "__main__":
    main()

