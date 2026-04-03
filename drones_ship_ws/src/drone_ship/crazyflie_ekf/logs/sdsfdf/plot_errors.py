#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Tunables
WARMUP_SEC = 2.0          # ignore first seconds after (re)start
MAX_DT     = 0.5          # drop samples with bigger gaps than this
MAX_SPEED  = 3.0          # absolute physical cap [m/s] (sim/cf ≤ ~2-3 m/s)
PCTL_CLIP  = 99.5         # also clip by percentile per series
ROLL_N     = 5            # rolling median window (points); set 0 to disable

def _prep_one(series: pd.DataFrame) -> pd.DataFrame:
    s = series.sort_values('time').copy()
    # compute step and dt
    dx  = s['x'].diff()
    dy  = s['y'].diff()
    step = np.hypot(dx, dy)
    dt   = s['time'].astype('int64').diff() / 1e9  # ns->s
    s['dt']   = dt
    s['step'] = step
    s['speed'] = step / dt

    # warm-up and bad-dt mask
    t0 = s['time'].iloc[0]
    mask = (dt > 0) & (dt <= MAX_DT) & ((s['time'] - t0).dt.total_seconds() >= WARMUP_SEC)

    # clip by percentile per series (after dt filter)
    sp = s.loc[mask, 'speed']
    if not sp.empty:
        pclip = np.percentile(sp.values, PCTL_CLIP)
        lim = min(MAX_SPEED, pclip)
        mask &= (s['speed'] <= lim)

    s['valid'] = mask
    if ROLL_N and s['valid'].any():
        s.loc[s['valid'], 'speed_med'] = s.loc[s['valid'], 'speed'].rolling(ROLL_N, min_periods=1).median()
    else:
        s['speed_med'] = np.nan
    return s

def plot_xy_speed(df_pose: pd.DataFrame):
    # df_pose columns: time (datetime64[ns, tz]), x, y, drone, src ('raw'|'ekf')
    fig, ax = plt.subplots(figsize=(13,6))

    drones = sorted(df_pose['drone'].astype(str).unique(),
                    key=lambda d: int(''.join(filter(str.isdigit, d)) or 0))
    styles = {'raw':'--', 'ekf':'-'}
    labels_done = set()

    for d in drones:
        for src in ['raw','ekf']:
            sub = df_pose[(df_pose['drone']==d) & (df_pose['src']==src)]
            if sub.empty: 
                continue
            s = _prep_one(sub)

            # plot filtered speeds
            v = s[s['valid']]
            if v.empty:
                continue
            lbl = f"{d} {src}"
            ax.plot(v['time'], v['speed'], styles.get(src,'-'),
                    alpha=0.35, linewidth=1.0, label=None)

            # overlay rolling median as the legended line
            if v['speed_med'].notna().any():
                ax.plot(v['time'], v['speed_med'], styles.get(src,'-'),
                        linewidth=2.0, label=lbl)

    ax.set_title("Odom vs EKF XY speed (outliers filtered)")
    ax.set_xlabel("time [s]" if df_pose['time'].dtype.kind!='M' else "time")
    ax.set_ylabel("speed in XY [m/s]")
    ax.grid(True, axis='y', alpha=0.3)
    ax.legend(loc='upper right', ncol=2, framealpha=0.9)
    plt.tight_layout()
    plt.show()
