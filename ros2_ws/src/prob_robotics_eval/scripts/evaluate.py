#!/usr/bin/env python3
"""
Offline evaluation of the CSV written by trajectory_logger.

Usage:
  python3 evaluate.py /tmp/trajectory_log.csv --tag baseline
  python3 evaluate.py /tmp/log_wrong_init.csv --tag wrong_init --out /tmp/plots/
"""
import argparse, csv, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FILTERS = ['odom', 'kf', 'ekf', 'pf']

def load_csv(path):
    rows = []
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        for r in reader:
            try:
                rows.append({k: float(v) if v != 'nan' else math.nan for k, v in r.items()})
            except ValueError:
                continue
    return rows

def compute_errors(rows):
    errors = {f: {'t': [], 'ex': [], 'ey': [], 'eyaw': [], 'epos': []} for f in FILTERS}
    for r in rows:
        gx, gy, gyaw = r['gt_x'], r['gt_y'], r['gt_yaw']
        if math.isnan(gx):
            continue
        for f in FILTERS:
            fx   = r.get(f + '_x',   math.nan)
            fy   = r.get(f + '_y',   math.nan)
            fyaw = r.get(f + '_yaw', math.nan)
            if math.isnan(fx):
                continue
            ex   = fx - gx
            ey   = fy - gy
            eyaw = math.atan2(math.sin(fyaw - gyaw), math.cos(fyaw - gyaw))
            errors[f]['t'].append(r['t'])
            errors[f]['ex'].append(ex)
            errors[f]['ey'].append(ey)
            errors[f]['eyaw'].append(eyaw)
            errors[f]['epos'].append(math.hypot(ex, ey))
    return errors

def rmse(values):
    if not values: return float('nan')
    return float(np.sqrt(np.mean(np.array(values) ** 2)))

def convergence_time(times, errs, threshold=0.2, sustain_sec=2.0):
    if not times: return float('nan')
    t = np.array(times); e = np.array(errs); n = len(t)
    for i in range(n):
        if e[i] < threshold:
            j = i
            while j < n and t[j] - t[i] < sustain_sec:
                if e[j] >= threshold: break
                j += 1
            else:
                return float(t[i])
            if j > 0 and t[j-1] - t[i] >= sustain_sec:
                return float(t[i])
    return float('nan')

def plot_trajectories(rows, out_path, tag):
    fig, ax = plt.subplots(figsize=(7, 7))
    gx = [r['gt_x'] for r in rows if not math.isnan(r['gt_x'])]
    gy = [r['gt_y'] for r in rows if not math.isnan(r['gt_x'])]
    ax.plot(gx, gy, 'k-', lw=2, label='Ground Truth')
    colors = {'odom': 'gray', 'kf': 'tab:blue', 'ekf': 'tab:green', 'pf': 'tab:red'}
    for f in FILTERS:
        fx = [r[f+'_x'] for r in rows if not math.isnan(r.get(f+'_x', math.nan))]
        fy = [r[f+'_y'] for r in rows if not math.isnan(r.get(f+'_x', math.nan))]
        if fx:
            ax.plot(fx, fy, '-', color=colors[f], lw=1.2, alpha=0.85, label=f.upper())
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title(f'Trajectories ({tag})')
    ax.axis('equal'); ax.grid(True); ax.legend()
    fig.savefig(os.path.join(out_path, f'trajectories_{tag}.png'), dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'  -> trajectories_{tag}.png')

def plot_errors_over_time(errors, out_path, tag):
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    colors = {'odom': 'gray', 'kf': 'tab:blue', 'ekf': 'tab:green', 'pf': 'tab:red'}
    for f in FILTERS:
        e = errors[f]
        if not e['t']: continue
        axes[0].plot(e['t'], e['epos'], color=colors[f], label=f.upper())
        axes[1].plot(e['t'], e['ex'],   color=colors[f], label=f.upper())
        axes[2].plot(e['t'], e['eyaw'], color=colors[f], label=f.upper())
    axes[0].set_ylabel('Position error [m]'); axes[0].grid(True); axes[0].legend()
    axes[1].set_ylabel('Error X [m]');         axes[1].grid(True)
    axes[2].set_ylabel('Error yaw [rad]');     axes[2].grid(True)
    axes[2].set_xlabel('t [s]')
    axes[0].set_title(f'Error over time ({tag})')
    fig.savefig(os.path.join(out_path, f'errors_{tag}.png'), dpi=130, bbox_inches='tight')
    plt.close(fig)
    print(f'  -> errors_{tag}.png')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_path')
    ap.add_argument('--tag',       default='run')
    ap.add_argument('--out',       default='/tmp/plots')
    ap.add_argument('--threshold', type=float, default=0.2)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    rows = load_csv(args.csv_path)
    if not rows:
        print('Empty CSV or parse error.'); sys.exit(1)

    errors = compute_errors(rows)

    print('\n=== RMSE ===')
    print(f'{"Filter":<8} {"RMSE_x":>9} {"RMSE_y":>9} {"RMSE_pos":>10} {"RMSE_yaw":>9} {"t_conv[s]":>10}')
    for f in FILTERS:
        e = errors[f]
        print(f'{f:<8} {rmse(e["ex"]):>9.4f} {rmse(e["ey"]):>9.4f} '
              f'{rmse(e["epos"]):>10.4f} {rmse(e["eyaw"]):>9.4f} '
              f'{convergence_time(e["t"], e["epos"], args.threshold):>10.2f}')

    print(f'\nSaving plots to {args.out}/')
    plot_trajectories(rows, args.out, args.tag)
    plot_errors_over_time(errors, args.out, args.tag)

if __name__ == '__main__':
    main()
