#!/usr/bin/env python3
"""Analyze Manual Return tracking logs and produce cross-track / along-track
error decomposition, plus an aggregated multi-run summary CSV.

Only the Python standard library is used (no numpy/pandas dependency).

Usage:
  python3 analyze_tracking_runs.py /tmp/manual_return/20260820_161439
  python3 analyze_tracking_runs.py --base /tmp/manual_return
  python3 analyze_tracking_runs.py --base /tmp/manual_return --out aggregated.csv
"""
import argparse
import csv
import glob
import math
import os
import sys


def read_rdp_waypoints(path):
    """Read return_path.csv (or legacy rdp_return_path.csv) -> list of (x, y, z)."""
    waypoints = []
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if not row:
                continue
            waypoints.append((float(row[1]), float(row[2]), float(row[3])))
    return waypoints


def read_tracking(path):
    """Read return_tracking_log.csv or return_tracking_analysis.csv.

    Returns list of dicts with keys t, ax, ay, az, rx, ry, rz, and optional
    phase / cross_track_3d if present.
    """
    samples = []
    with open(path) as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return samples
        idx = {name: i for i, name in enumerate(header)}
        for row in reader:
            if not row:
                continue
            s = {}
            if 'timestamp' in idx:
                s['t'] = float(row[idx['timestamp']])
            if 'actual_x' in idx:
                s['ax'] = float(row[idx['actual_x']])
                s['ay'] = float(row[idx['actual_y']])
                s['az'] = float(row[idx['actual_z']])
            if 'ref_x' in idx:
                s['rx'] = float(row[idx['ref_x']])
                s['ry'] = float(row[idx['ref_y']])
                s['rz'] = float(row[idx['ref_z']])
            if 'return_phase' in idx:
                s['phase'] = row[idx['return_phase']]
            if 'cross_track_3d' in idx:
                s['cross_track_3d'] = float(row[idx['cross_track_3d']])
            if 'cross_track_xy' in idx:
                s['cross_track_xy'] = float(row[idx['cross_track_xy']])
            if 'vertical_error' in idx:
                s['vertical_error'] = float(row[idx['vertical_error']])
            if 'along_track_error' in idx:
                s['along_track_error'] = float(row[idx['along_track_error']])
            if 'actual_progress_s' in idx:
                s['actual_progress_s'] = float(row[idx['actual_progress_s']])
            if 'reference_progress_s' in idx:
                s['reference_progress_s'] = float(row[idx['reference_progress_s']])
            samples.append(s)
    return samples


def _seg_len(a, b):
    return math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2 + (b[2]-a[2])**2)


def build_polyline(waypoints):
    cum = [0.0]
    total = 0.0
    for i in range(1, len(waypoints)):
        total += _seg_len(waypoints[i-1], waypoints[i])
        cum.append(total)
    return cum


def project_monotonic(p, waypoints, cum, last_seg):
    """Project point p onto polyline with monotonic (forward) segment search."""
    n = len(waypoints)
    if n < 2:
        return 0, waypoints[0] if waypoints else p, 0.0
    best_seg = max(0, min(last_seg, n - 2))
    max_seg = min(n - 2, best_seg + 3)
    best_dist = float('inf')
    best_u = 0.0
    best_q = waypoints[best_seg]
    for seg in range(best_seg, max_seg + 1):
        a = waypoints[seg]
        b = waypoints[seg + 1]
        v = (b[0]-a[0], b[1]-a[1], b[2]-a[2])
        vv = v[0]*v[0] + v[1]*v[1] + v[2]*v[2]
        u = 0.0
        if vv > 1e-12:
            u = ((p[0]-a[0])*v[0] + (p[1]-a[1])*v[1] + (p[2]-a[2])*v[2]) / vv
        u = max(0.0, min(1.0, u))
        q = (a[0]+u*v[0], a[1]+u*v[1], a[2]+u*v[2])
        d = math.sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2 + (p[2]-q[2])**2)
        if d < best_dist:
            best_dist = d
            best_seg = seg
            best_u = u
            best_q = q
    s = cum[best_seg] + best_u * (cum[best_seg+1] - cum[best_seg])
    return best_seg, best_q, s


def percentile(vals, p):
    if not vals:
        return 0.0
    if len(vals) == 1:
        return vals[0]
    sv = sorted(vals)
    pos = p * (len(sv) - 1)
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return sv[lo]
    frac = pos - lo
    return sv[lo] * (1 - frac) + sv[hi] * frac


def stats(vals):
    if not vals:
        return {'mean': 0.0, 'p50': 0.0, 'p90': 0.0, 'p95': 0.0, 'p99': 0.0, 'max': 0.0}
    return {
        'mean': sum(vals) / len(vals),
        'p50': percentile(vals, 0.50),
        'p90': percentile(vals, 0.90),
        'p95': percentile(vals, 0.95),
        'p99': percentile(vals, 0.99),
        'max': max(vals),
    }


def analyze_run(run_dir):
    # New format writes return_path.csv; older runs used rdp_return_path.csv.
    rdp_path = os.path.join(run_dir, 'return_path.csv')
    if not os.path.isfile(rdp_path):
        rdp_path = os.path.join(run_dir, 'rdp_return_path.csv')
    # V1.0.2 moved the analysis CSV under diagnostics/; fall back to the root.
    analysis_path = os.path.join(run_dir, 'diagnostics', 'return_tracking_analysis.csv')
    if not os.path.isfile(analysis_path):
        analysis_path = os.path.join(run_dir, 'return_tracking_analysis.csv')
    tracking_path = os.path.join(run_dir, 'return_tracking_log.csv')

    if not os.path.isfile(rdp_path):
        print("WARNING: %s has no return_path.csv" % run_dir, file=sys.stderr)
        return None
    waypoints = read_rdp_waypoints(rdp_path)
    cum = build_polyline(waypoints)

    # Prefer the full tracking log (has actual_x/y/z + cross-track columns).
    # The diagnostics/return_tracking_analysis.csv is a compact subset without
    # actual positions, so it is only used as a fallback.
    if os.path.isfile(tracking_path):
        samples = read_tracking(tracking_path)
    elif os.path.isfile(analysis_path):
        samples = read_tracking(analysis_path)
    else:
        print("WARNING: %s has no tracking log" % run_dir, file=sys.stderr)
        return None

    # If the log already has cross-track columns, use them; otherwise compute
    # cross-track / along-track by projecting actual onto the planned polyline.
    has_precomputed = all('cross_track_3d' in s for s in samples) and samples

    time_err = []
    cross3d = []
    cross_xy = []
    vertical = []
    along = []
    last_seg = 0

    for s in samples:
        ax, ay, az = s.get('ax'), s.get('ay'), s.get('az')
        if ax is None:
            continue
        # time-synchronized error (only when ref is available)
        if s.get('rx') is not None:
            time_err.append(math.sqrt(
                (s['rx']-ax)**2 + (s['ry']-ay)**2 + (s['rz']-az)**2))
        if has_precomputed:
            cross3d.append(s['cross_track_3d'])
            if 'cross_track_xy' in s:
                cross_xy.append(s['cross_track_xy'])
            if 'vertical_error' in s:
                vertical.append(s['vertical_error'])
            if 'along_track_error' in s:
                along.append(s['along_track_error'])
        else:
            seg, q, sp = project_monotonic((ax, ay, az), waypoints, cum, last_seg)
            last_seg = seg
            c3 = math.sqrt((ax-q[0])**2 + (ay-q[1])**2 + (az-q[2])**2)
            cxy = math.sqrt((ax-q[0])**2 + (ay-q[1])**2)
            vz = abs(az - q[2])
            cross3d.append(c3)
            cross_xy.append(cxy)
            vertical.append(vz)
            # reference progress: project ref onto polyline
            if s.get('rx') is not None:
                _, _, sr = project_monotonic((s['rx'], s['ry'], s['rz']), waypoints, cum, 0)
                along.append(sr - sp)

    scenario = ""
    metrics_path = os.path.join(run_dir, "return_metrics.csv")
    if os.path.isfile(metrics_path):
        with open(metrics_path) as mf:
            mr = csv.DictReader(mf)
            row = next(mr, None)
            if row:
                scenario = row.get("scenario", "")

    result = {
        'run_id': os.path.basename(run_dir.rstrip('/')),
        'scenario': scenario,
        'samples': len(samples),
        'rdp_points': len(waypoints),
        'return_length': cum[-1] if cum else 0.0,
        'time': stats(time_err),
        'cross3d': stats(cross3d),
        'cross_xy': stats(cross_xy),
        'vertical': stats(vertical),
        'along': stats(along),
        'final_home_error': 0.0,
    }
    # final home error: distance from last actual to last waypoint
    if samples and waypoints:
        last = samples[-1]
        if last.get('ax') is not None:
            hx, hy, hz = waypoints[-1]
            result['final_home_error'] = math.sqrt(
                (last['ax']-hx)**2 + (last['ay']-hy)**2 + (last['az']-hz)**2)
    return result


def discover_runs(base):
    runs = []
    for entry in sorted(os.listdir(base)):
        d = os.path.join(base, entry)
        if os.path.isdir(d) and (os.path.isfile(os.path.join(d, 'return_path.csv')) or
                                 os.path.isfile(os.path.join(d, 'rdp_return_path.csv'))):
            runs.append(d)
    return runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='*', help='run directories to analyze')
    ap.add_argument('--base', help='parent directory; discover all run subdirs')
    ap.add_argument('--out', default='aggregated_tracking_summary.csv')
    args = ap.parse_args()

    runs = list(args.runs)
    if args.base:
        runs += discover_runs(args.base)
    if not runs:
        print("No run directories given. Use positional dirs or --base.", file=sys.stderr)
        return 2

    rows = []
    for run_dir in runs:
        res = analyze_run(run_dir)
        if res is None:
            print("SKIP (no return_path.csv):", run_dir, file=sys.stderr)
            continue
        rows.append(res)
        print("=" * 70)
        print("run_id            :", res['run_id'])
        print("samples           :", res['samples'])
        print("rdp_points        :", res['rdp_points'])
        print("return_length     : %.3f m" % res['return_length'])
        print("time-sync error   : mean=%.4f P95=%.4f max=%.4f m"
              % (res['time']['mean'], res['time']['p95'], res['time']['max']))
        print("cross-track 3D    : mean=%.4f P50=%.4f P90=%.4f P95=%.4f P99=%.4f max=%.4f m"
              % (res['cross3d']['mean'], res['cross3d']['p50'], res['cross3d']['p90'],
                 res['cross3d']['p95'], res['cross3d']['p99'], res['cross3d']['max']))
        print("cross-track XY    : mean=%.4f P95=%.4f max=%.4f m"
              % (res['cross_xy']['mean'], res['cross_xy']['p95'], res['cross_xy']['max']))
        print("vertical error    : mean=%.4f P95=%.4f max=%.4f m"
              % (res['vertical']['mean'], res['vertical']['p95'], res['vertical']['max']))
        print("along-track error : mean_abs=%.4f P95_abs=%.4f max_abs=%.4f m"
              % (res['along']['mean'], res['along']['p95'], res['along']['max']))
        print("final_home_error  : %.4f m" % res['final_home_error'])

    # Write aggregated CSV
    with open(args.out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow([
            'run_id', 'scenario', 'samples', 'rdp_points', 'distance',
            'mean_time_error', 'p95_time_error', 'max_time_error',
            'mean_cross_track', 'p95_cross_track', 'p99_cross_track', 'max_cross_track',
            'p95_horizontal', 'p95_vertical',
            'mean_along_track_abs', 'p95_along_track_abs',
            'final_home_error',
        ])
        for r in rows:
            w.writerow([
                r['run_id'], r['scenario'], r['samples'], r['rdp_points'], round(r['return_length'], 4),
                round(r['time']['mean'], 4), round(r['time']['p95'], 4), round(r['time']['max'], 4),
                round(r['cross3d']['mean'], 4), round(r['cross3d']['p95'], 4),
                round(r['cross3d']['p99'], 4), round(r['cross3d']['max'], 4),
                round(r['cross_xy']['p95'], 4), round(r['vertical']['p95'], 4),
                round(r['along']['mean'], 4), round(r['along']['p95'], 4),
                round(r['final_home_error'], 4),
            ])
    print("=" * 70)
    print("wrote aggregated summary ->", args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
