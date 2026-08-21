#!/usr/bin/env python3
"""Aggregate per-run return_metrics.csv into a single comparison CSV.

Reads every run subdirectory under a base directory, parses each
`return_metrics.csv`, and writes `return_all_runs_summary.csv` with one row per
run (plus an optional user-editable `scenario` column).  This is the common
benchmark format used to compare different return algorithms.

Only the Python standard library is used.

Usage:
  python3 analyze_return_runs.py --base ~/fanhang/logs/manual_return_output
  python3 analyze_return_runs.py --base ~/fanhang/logs/manual_return_output \
      --out return_all_runs_summary.csv
"""
import argparse
import csv
import os
import sys

# Must match the column order of return_metrics.csv.
METRIC_FIELDS = [
    "run_id", "original_points", "original_length_m", "simplified_points",
    "point_reduction_percent", "return_length_m", "length_change_percent",
    "max_deviation_m", "mean_deviation_m", "p95_deviation_m",
    "min_clearance_m", "clearance_available", "unsafe_segments",
    "validated_segments", "collision_check_count", "cross_track_p95",
    "cross_track_max", "along_track_p95", "vertical_error_p95",
    "final_home_error_m", "return_duration_s", "planning_time_ms",
    "memory_usage_mb", "pointcloud_size", "voxelized_cloud_size",
]


def discover_runs(base):
    runs = []
    if not os.path.isdir(base):
        return runs
    for entry in sorted(os.listdir(base)):
        d = os.path.join(base, entry)
        metrics = os.path.join(d, "return_metrics.csv")
        if os.path.isdir(d) and os.path.isfile(metrics):
            runs.append(d)
    return runs


def read_metrics(path):
    with open(path) as f:
        reader = csv.DictReader(f)
        return next(reader, None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True,
                    help="parent dir containing run subdirectories")
    ap.add_argument("--out", default="return_all_runs_summary.csv")
    args = ap.parse_args()

    runs = discover_runs(args.base)
    if not runs:
        print("No run directories with return_metrics.csv under",
              args.base, file=sys.stderr)
        return 2

    out_fields = ["scenario"] + METRIC_FIELDS
    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=out_fields)
        w.writeheader()
        for d in runs:
            row = read_metrics(os.path.join(d, "return_metrics.csv"))
            if row is None:
                continue
            out = {"scenario": row.get("scenario", "")}
            for k in METRIC_FIELDS:
                out[k] = row.get(k, "")
            w.writerow(out)

    print("wrote %d runs -> %s" % (len(runs), args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
