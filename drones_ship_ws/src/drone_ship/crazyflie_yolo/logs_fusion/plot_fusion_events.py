import os
import glob
import re
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

# =========================
# Container palette + labels
# =========================
CONTAINER_PALETTE = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]  # C1..C5

ALPHA_RAW = 1.00
ALPHA_REMAINING = 0.65
ALPHA_PRUNED = 0.35

def container_label(name: str) -> str:
    """container1 -> C1, container2 -> C2, ..."""
    m = re.search(r"container\s*(\d+)", str(name), flags=re.I)
    return f"C{m.group(1)}" if m else str(name)

def build_map(keys_in_order, palette):
    # cycles if more keys than colors
    colors = [palette[i % len(palette)] for i in range(len(keys_in_order))]
    return dict(zip(keys_in_order, colors))

# -----------------------------
# Load data: any *fusion_events*.csv in current directory
# -----------------------------
pattern = "*fusion_events*.csv"
files = sorted(glob.glob(pattern))

# case-insensitive fallback
if not files:
    files = sorted([f for f in glob.glob("*.csv") if "fusion_events" in os.path.basename(f).lower()])

if not files:
    raise RuntimeError(f"No CSV files found in current directory matching '{pattern}' (case-insensitive).")

print(f"[INFO] Found {len(files)} fusion_events file(s):")
for f in files:
    print("  -", f)

dfs = []
for f in files:
    d = pd.read_csv(f)
    d["__source_file__"] = os.path.basename(f)
    dfs.append(d)

ev = pd.concat(dfs, ignore_index=True)

# -----------------------------
# Rename containers to C1..C5
# -----------------------------
if "class_name" not in ev.columns:
    raise RuntimeError("Missing required column: 'class_name'")
ev["class_label"] = ev["class_name"].apply(container_label)

# -----------------------------------------
# Counts per drone per class:
#   raw       = number of events (rows)
#   remaining = unique associated tracks kept after pruning
#   pruned    = raw - remaining
# -----------------------------------------
required = {"src_drone_ns", "associated_track_id"}
missing = [c for c in required if c not in ev.columns]
if missing:
    raise RuntimeError(f"Missing required column(s): {missing}")

raw = ev.groupby(["src_drone_ns", "class_label"]).size().unstack(fill_value=0)

remaining = (
    ev.groupby(["src_drone_ns", "class_label"])["associated_track_id"]
      .nunique()
      .unstack(fill_value=0)
)

pruned = (raw - remaining).clip(lower=0)

# --- Align indices/columns across tables ---
drones = sorted(set(raw.index).union(set(remaining.index)).union(set(pruned.index)))

def class_sort_key(s: str) -> int:
    m = re.search(r"\d+", str(s))
    return int(m.group()) if m else 999

classes = sorted(
    set(raw.columns).union(set(remaining.columns)).union(set(pruned.columns)),
    key=class_sort_key
)

raw = raw.reindex(index=drones, columns=classes, fill_value=0)
remaining = remaining.reindex(index=drones, columns=classes, fill_value=0)
pruned = pruned.reindex(index=drones, columns=classes, fill_value=0)

# -----------------------------------------
# Container color map (C1..C5)
# -----------------------------------------
container_color_map = build_map(classes, CONTAINER_PALETTE)

# -----------------------------------------
# Plot: grouped bars (3 bars per class)
# -----------------------------------------
x = np.arange(len(drones))
n_classes = len(classes)

bar_w = 0.16
bars_per_class = 3
class_block = bars_per_class * bar_w
group_w = n_classes * class_block

# Scale to fit within ~0.9 width per drone group
scale = 0.9 / group_w if group_w > 0 else 1.0
bar_w *= scale
class_block = bars_per_class * bar_w
group_w = n_classes * class_block

fig, ax = plt.subplots(figsize=(12, 5.8))

# RAW (darkest)
for ci, cls in enumerate(classes):
    base = (-group_w / 2) + ci * class_block
    pos_raw = x + base
    ax.bar(
        pos_raw,
        raw[cls].values,
        width=bar_w,
        color=container_color_map[cls],
        alpha=ALPHA_RAW,
        edgecolor="black",
        linewidth=0.6,
        label=cls
    )

# PRUNED (lightest)
for ci, cls in enumerate(classes):
    base = (-group_w / 2) + ci * class_block
    pos_pruned = x + base + bar_w
    ax.bar(
        pos_pruned,
        pruned[cls].values,
        width=bar_w,
        color=container_color_map[cls],
        alpha=ALPHA_PRUNED,
        edgecolor="black",
        linewidth=0.6,
    )

# REMAINING (medium)
for ci, cls in enumerate(classes):
    base = (-group_w / 2) + ci * class_block
    pos_rem = x + base + 2 * bar_w
    ax.bar(
        pos_rem,
        remaining[cls].values,
        width=bar_w,
        color=container_color_map[cls],
        alpha=ALPHA_REMAINING,
        edgecolor="black",
        linewidth=0.6,
    )

# --- Labels / styling ---
ax.set_title("Raw vs pruned vs remaining obstacles per drone and container class")
ax.set_xlabel("Drone")
ax.set_ylabel("Count")
ax.set_xticks(x, drones)
ax.grid(True, axis="y", alpha=0.3)

# Legend 1: container classes (colors) -> C1..C5
class_handles = [
    Patch(facecolor=container_color_map[c], edgecolor="black", label=c, alpha=ALPHA_RAW)
    for c in classes
]
leg1 = ax.legend(handles=class_handles, title="Container class", loc="upper right")
ax.add_artist(leg1)

# Legend 2: bar meaning (alpha levels)
type_patches = [
    Patch(facecolor="gray", edgecolor="black", label="Raw (events)", alpha=ALPHA_RAW),
    Patch(facecolor="gray", edgecolor="black", label="Pruned/removed (raw - remaining)", alpha=ALPHA_PRUNED),
    Patch(facecolor="gray", edgecolor="black", label="Remaining after pruning (unique associated_track_id)", alpha=ALPHA_REMAINING),
]
ax.legend(handles=type_patches, title="Bar meaning (same color, different opacity)", loc="upper left")

plt.tight_layout()
plt.show()
