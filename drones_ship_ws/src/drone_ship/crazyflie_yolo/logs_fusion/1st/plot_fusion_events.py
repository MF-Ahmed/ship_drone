import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

# -----------------------------
# Load data: any *fusion_events*.csv in current directory
# -----------------------------
pattern = "*fusion_events*.csv"
files = sorted(glob.glob(pattern))

# (optional) also match case-insensitive by scanning all CSVs:
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

# -----------------------------------------
# Counts per drone per class:
#   raw       = number of events (rows)
#   remaining = unique associated tracks kept after pruning
#   pruned    = raw - remaining
# -----------------------------------------
raw = ev.groupby(["src_drone_ns", "class_name"]).size().unstack(fill_value=0)

remaining = (
    ev.groupby(["src_drone_ns", "class_name"])["associated_track_id"]
      .nunique()
      .unstack(fill_value=0)
)

pruned = (raw - remaining).clip(lower=0)

# --- Align indices/columns across tables ---
drones = sorted(set(raw.index).union(set(remaining.index)).union(set(pruned.index)))
classes = sorted(set(raw.columns).union(set(remaining.columns)).union(set(pruned.columns)))

raw = raw.reindex(index=drones, columns=classes, fill_value=0)
remaining = remaining.reindex(index=drones, columns=classes, fill_value=0)
pruned = pruned.reindex(index=drones, columns=classes, fill_value=0)

# -----------------------------------------
# Plot: grouped bars (3 bars per class)
# -----------------------------------------
x = np.arange(len(drones))
n_classes = len(classes)

# bar widths: 3 bars per class (raw, pruned, remaining)
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

# Draw RAW bars first and capture class colors
class_colors = {}
for ci, cls in enumerate(classes):
    base = (-group_w / 2) + ci * class_block
    pos_raw = x + base
    bars = ax.bar(pos_raw, raw[cls].values, width=bar_w, label=cls)
    class_colors[cls] = bars[0].get_facecolor() if len(bars) else None

# Draw PRUNED (removed) bars next to raw, same color, hatched
for ci, cls in enumerate(classes):
    base = (-group_w / 2) + ci * class_block
    pos_pruned = x + base + bar_w
    ax.bar(
        pos_pruned,
        pruned[cls].values,
        width=bar_w,
        color=class_colors[cls],
        hatch="//",
        alpha=0.9,
    )

# Draw REMAINING (kept) bars next, same color, different hatch
for ci, cls in enumerate(classes):
    base = (-group_w / 2) + ci * class_block
    pos_rem = x + base + 2 * bar_w
    ax.bar(
        pos_rem,
        remaining[cls].values,
        width=bar_w,
        color=class_colors[cls],
        hatch="..",
        alpha=0.9,
    )

# --- Labels / styling ---
ax.set_title("Raw vs pruned vs remaining obstacles per drone and container class")
ax.set_xlabel("Drone")
ax.set_ylabel("Count")
ax.set_xticks(x, drones)
ax.grid(True, axis="y", alpha=0.3)

# Legend 1: class colors
handles, labels = ax.get_legend_handles_labels()
seen = set()
h2, l2 = [], []
for h, l in zip(handles, labels):
    if l not in seen:
        seen.add(l)
        h2.append(h)
        l2.append(l)
leg1 = ax.legend(h2, l2, title="class_name", loc="upper right")
ax.add_artist(leg1)

# Legend 2: bar meaning (hatches)
type_patches = [
    Patch(facecolor="white", edgecolor="black", label="Raw (events)", hatch=""),
    Patch(facecolor="white", edgecolor="black", label="Pruned/removed (raw - remaining)", hatch="//"),
    Patch(facecolor="white", edgecolor="black", label="Remaining after pruning (unique associated_track_id)", hatch=".."),
]
ax.legend(handles=type_patches, title="bar meaning", loc="upper left")

ax.text(
    0.01,
    0.98,
    "Per class (same color):\n"
    "  solid = Raw events\n"
    "  //    = Pruned/removed (raw - remaining)\n"
    "  ..    = Remaining after pruning (unique associated tracks)",
    transform=ax.transAxes,
    va="top",
)

plt.tight_layout()
plt.show()
