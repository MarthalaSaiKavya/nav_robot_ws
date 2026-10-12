"""
run_report - turn a run folder into a human-readable summary (and plots if matplotlib exists).

    ros2 run my_bot_autonomy run_report ~/nav_robot_runs/20260101_120000
    ros2 run my_bot_autonomy run_report            # newest run

Reads whatever is present in the folder:
    run_summary.json         (supervisor: total wall-clock time, timeline)
    exploration_result.json  (explorer)
    map_quality.json         (supervisor / compare_maps)
    mission_results.json     (mission_runner)
    evaluation_summary.json  (evaluator)
    pose_log.csv             (evaluator)  -> trajectory.png, errors.png
Writes summary.md next to them and returns the text.
"""
import csv
import glob
import json
import os
import sys

from ..common.ros_utils import default_run_root


def _load(run_dir, name):
    path = os.path.join(run_dir, name)
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            return json.load(f)
    except ValueError:
        return None


def _plots(run_dir):
    path = os.path.join(run_dir, 'pose_log.csv')
    if not os.path.exists(path):
        return []
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except Exception:
        return []
    cols = {}
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            for k, v in row.items():
                cols.setdefault(k, []).append(float(v) if v not in ('', None) else float('nan'))
    if not cols.get('t'):
        return []
    out = []
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(cols['truth_x'], cols['truth_y'], 'g-', lw=2, label='ground truth')
    ax.plot(cols['est_x'], cols['est_y'], 'y--', lw=1.2, label='estimate (map->base)')
    ax.plot(cols['odom_x'], cols['odom_y'], 'r:', lw=1, label='odometry only')
    ax.set_aspect('equal')
    ax.set_xlim(-5.5, 5.5)
    ax.set_ylim(-5.5, 5.5)
    ax.grid(alpha=0.3)
    ax.legend(loc='upper right')
    ax.set_title('Robot trajectory')
    p = os.path.join(run_dir, 'trajectory.png')
    fig.savefig(p, dpi=110, bbox_inches='tight')
    plt.close(fig)
    out.append(p)
    fig, ax = plt.subplots(figsize=(9, 4))
    t0 = cols['t'][0]
    t = [v - t0 for v in cols['t']]
    ax.plot(t, cols['loc_error_m'], label='localization error [m]')
    ax.plot(t, cols['odom_drift_m'], label='odometry drift [m]', alpha=0.7)
    ax.set_xlabel('time [s]')
    ax.set_ylabel('error [m]')
    ax.grid(alpha=0.3)
    ax.legend()
    ax.set_title('Pose errors vs ground truth')
    p = os.path.join(run_dir, 'errors.png')
    fig.savefig(p, dpi=110, bbox_inches='tight')
    plt.close(fig)
    out.append(p)
    return out


def make_report(run_dir):
    run = _load(run_dir, 'run_summary.json') or {}
    explo = _load(run_dir, 'exploration_result.json')
    mq = _load(run_dir, 'map_quality.json')
    mission = _load(run_dir, 'mission_results.json')
    ev = _load(run_dir, 'evaluation_summary.json')
    lines = [f'# Run report - {os.path.basename(os.path.normpath(run_dir))}', '']
    if run:
        ok = run.get('success')
        lines += [f'**Result: {"SUCCESS - all goals complete" if ok else "NOT COMPLETE"}**', '',
                  f'**Total wall-clock time (launch -> all goals complete): {run.get("total_wall_clock_s")} s**', '']
    if explo:
        lines += ['## Stage 1 - exploration + mapping', '',
                  f'- strategy: `{explo.get("strategy")}`, duration {explo.get("duration_s")} s',
                  f'- frontier goals sent {explo.get("goals_sent")}: reached {explo.get("goals_reached")}, '
                  f'seen-before-arrival {explo.get("targets_explored_early", 0)}, failed {explo.get("goals_failed")}; '
                  f'blacklisted spots: {explo.get("blacklisted")}',
                  f'- distance driven: {explo.get("distance_travelled_m", 0):.1f} m, '
                  f'returned home: {explo.get("returned_home")}',
                  f'- map: `{explo.get("map_yaml")}` (saved with `{explo.get("map_save_method")}`), '
                  f'pose graph saved: {explo.get("pose_graph_saved")}', '']
    if mq:
        lines += ['## Map quality vs. the true world', '',
                  '| metric | value | target |', '|---|---|---|',
                  f'| wall error mean | {mq["wall_error_mean"]:.3f} m | < 0.03 m |',
                  f'| wall error p95 | {mq["wall_error_p95"]:.3f} m | <= 0.10 m |',
                  f'| ghost walls | {100 * mq["ghost_wall_ratio"]:.1f} % | <= 2 % |',
                  f'| wall recall | {100 * mq["wall_recall"]:.1f} % | high |',
                  f'| coverage | {100 * mq["coverage"]:.1f} % | >= 90 % |',
                  f'| **pass** | **{mq["pass"]}** | |', '']
    if mission and 'summary' in mission:
        s = mission['summary']
        lines += ['## Stage 2 - localization + mission', '',
                  f'- method `{s["method"]}`: {s["succeeded"]}/{s["goals"]} goals, mission time {s["mission_time_s"]} s', '',
                  '| # | goal | attempt | result | time [s] | recoveries | dist [m] | final err [m] | loc err [m] |',
                  '|---|---|---|---|---|---|---|---|---|']
        for g in mission.get('goals', []):
            lines.append(f'| {g["index"]} | {g["name"]} | {g["attempt"]} | {g["result"]} | {g["duration_s"]} | '
                         f'{g.get("recoveries", "")} | {g.get("distance_m", "")} | '
                         f'{g.get("final_position_error_m", "")} | {g.get("localization_error_m", "")} |')
        lines.append('')
    if ev:
        le, od = ev.get('localization_error_m') or {}, ev.get('odometry_drift_m') or {}
        lines += ['## Localization accuracy (whole run, vs ground truth)', '',
                  f'- localization error: mean {le.get("mean")} m, p95 {le.get("p95")} m, max {le.get("max")} m',
                  f'- odometry-only drift: mean {od.get("mean")} m, max {od.get("max")} m  '
                  '(this is what SLAM/AMCL fixes)',
                  f'- near-collisions detected: {ev.get("near_collisions")}', '']
    if run.get('timeline'):
        lines += ['## Timeline', '', '| t [s] | event |', '|---|---|']
        lines += [f'| {e["t"]} | {e["what"]} |' for e in run['timeline']]
        lines.append('')
    plots = _plots(run_dir)
    for pth in plots:
        lines.append(f'![{os.path.basename(pth)}]({os.path.basename(pth)})')
    text = '\n'.join(lines)
    with open(os.path.join(run_dir, 'summary.md'), 'w') as f:
        f.write(text + '\n')
    return text


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    argv = [a for a in argv if not a.startswith('--ros-args')]
    if argv:
        run_dir = os.path.expanduser(argv[0])
    else:
        runs = sorted(glob.glob(os.path.join(default_run_root(), '*')))
        if not runs:
            print('no runs found in', default_run_root())
            return
        run_dir = runs[-1]
    print(make_report(run_dir))
    print(f'\n(written to {run_dir}/summary.md)')


if __name__ == '__main__':
    main()
