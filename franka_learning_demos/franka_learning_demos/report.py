"""Generate shareable figures and metrics from one recorded circle run."""
import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def read_csv(path):
    with path.open(newline='') as stream:
        reader = csv.reader(stream)
        header = next(reader)
        rows = [[float(value) for value in row] for row in reader]
    return np.array(rows, dtype=float).reshape((-1, len(header)))


def write_report(directory):
    directory = Path(directory)
    summary_path = directory / 'summary.json'
    summary = json.loads(summary_path.read_text())
    target = read_csv(directory / 'target.csv')
    planned = read_csv(directory / 'planned.csv')
    tracking = read_csv(directory / 'tracking.csv')
    if len(tracking):
        errors = tracking[:, 8]
        summary['metrics'] = {
            'position_rmse_m': float(np.sqrt(np.mean(errors**2))),
            'position_max_m': float(np.max(errors)),
            'position_p95_m': float(np.percentile(errors, 95)),
            'orientation_max_rad': float(np.max(tracking[:, 9])),
            'max_sample_gap_s': float(np.max(np.diff(tracking[:, 1]))) if len(tracking) > 1 else None,
        }
        if 'center_m' in summary:
            center = np.array(summary['center_m'])
            radius = summary['config']['radius']
            delta = tracking[:, 5:8] - center
            geometric_error = np.hypot(np.linalg.norm(delta[:, :2], axis=1) - radius, delta[:, 2])
            summary['metrics']['circle_distance_rmse_m'] = float(np.sqrt(np.mean(geometric_error**2)))
        if len(target):
            summary['metrics']['final_target_error_m'] = float(np.linalg.norm(tracking[-1, 5:8] - target[-1]))
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    if not len(target):
        return
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig = plt.figure(figsize=(13, 8), layout='constrained')
    grid = fig.add_gridspec(2, 2)
    xyz = fig.add_subplot(grid[:, 0], projection='3d')
    xy = fig.add_subplot(grid[0, 1])
    error_axes = fig.add_subplot(grid[1, 1])
    origin = target[0]
    curves = [('Requested circle', target, '#718096', '--')]
    if len(planned):
        curves.append(('Planned (time parameterized)', planned[:, 1:4], '#d69e2e', ':'))
    if len(tracking):
        curves.extend([('Controller reference', tracking[:, 2:5], '#147d92', '-'),
                       ('Measured from joint feedback', tracking[:, 5:8], '#dd6b20', '-')])
    for label, values, color, style in curves:
        relative = 1000 * (values - origin)
        xyz.plot(*relative.T, label=label, color=color, linestyle=style, linewidth=1.7)
        xy.plot(relative[:, 0], relative[:, 1], label=label, color=color, linestyle=style, linewidth=1.7)
    xyz.set(xlabel='X offset (mm)', ylabel='Y offset (mm)', zlabel='Z offset (mm)')
    radius_mm = summary['config']['radius'] * 1000
    xyz.set_xlim(-2.2*radius_mm, 0.2*radius_mm)
    xyz.set_ylim(-1.2*radius_mm, 1.2*radius_mm)
    xyz.set_zlim(-1.2*radius_mm, 1.2*radius_mm)
    xyz.set_box_aspect((1, 1, 1))
    xyz.view_init(elev=35, azim=-65)
    xyz.legend(loc='upper left', fontsize=8)
    xy.set(xlabel='X offset (mm)', ylabel='Y offset (mm)', title='Circle in base-frame XY plane')
    xy.set_aspect('equal', adjustable='datalim')
    xy.grid(alpha=0.25)
    if len(tracking):
        error_axes.plot(tracking[:, 1], tracking[:, 8] * 1000, color='#dd6b20')
        metrics = summary['metrics']
        error_axes.set_title(f"Tracking RMSE {metrics['position_rmse_m']*1000:.3f} mm | max {metrics['position_max_m']*1000:.3f} mm")
    else:
        error_axes.text(0.5, 0.5, 'No execution samples', ha='center', transform=error_axes.transAxes)
    error_axes.set(xlabel='Controller trajectory time (simulation s)', ylabel='Position error (mm)')
    error_axes.grid(alpha=0.25)
    fig.suptitle(f"Panda circle demo | {summary['status']} | {summary['tip']} in {summary['frame']}\n"
                 'Simulation feedback and URDF forward kinematics; not external positioning accuracy', fontsize=13)
    fig.savefig(directory / 'trajectory.png', dpi=170)
    fig.savefig(directory / 'trajectory.svg')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    write_report(args.directory)
    print(args.directory / 'trajectory.png')
