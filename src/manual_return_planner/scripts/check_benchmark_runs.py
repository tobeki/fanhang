#!/usr/bin/env python3
"""Check benchmark run directories for completeness and field sanity.

Scans each run directory under --base, verifies the required files exist, and
checks the presence/validity of the key metric fields.  Writes a human-readable
report to benchmark_check_report.txt (and prints a summary).

Usage:
  python3 check_benchmark_runs.py --base ~/fanhang/logs/manual_return_output
"""
import argparse
import csv
import os
import sys

REQUIRED_FILES = [
    "return_metrics.csv",
    "return_tracking_log.csv",
    "return_summary.txt",
]

KEY_FIELDS = [
    "scenario",
    "original_points",
    "simplified_points",
    "max_deviation_m",
    "cross_track_p95",
]


def check_run(run_dir):
    """Return (ok, problems)."""
    problems = []
    files = {}
    for name in REQUIRED_FILES:
        p = os.path.join(run_dir, name)
        files[name] = os.path.isfile(p)
        if not files[name]:
            problems.append("missing file: %s" % name)

    metrics = {}
    if files["return_metrics.csv"]:
        with open(os.path.join(run_dir, "return_metrics.csv")) as f:
            reader = csv.DictReader(f)
            row = next(reader, None)
            if row is None:
                problems.append("return_metrics.csv is empty")
            else:
                metrics = row

    for field in KEY_FIELDS:
        if field not in metrics:
            problems.append("missing field: %s" % field)
            continue
        v = metrics[field]
        if v is None or (isinstance(v, str) and v.strip() == ""):
            problems.append("empty field: %s" % field)

    return len(problems) == 0, problems, metrics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True)
    ap.add_argument("--out", default="benchmark_check_report.txt")
    args = ap.parse_args()

    base = args.base
    if not os.path.isdir(base):
        print("base dir not found:", base, file=sys.stderr)
        return 2

    runs = []
    for entry in sorted(os.listdir(base)):
        d = os.path.join(base, entry)
        if os.path.isdir(d) and os.path.isfile(os.path.join(d, "return_metrics.csv")):
            runs.append(d)

    lines = []
    lines.append("Benchmark check report")
    lines.append("base: %s" % base)
    lines.append("runs: %d" % len(runs))
    lines.append("=" * 60)

    n_ok = 0
    for run_dir in runs:
        ok, problems, metrics = check_run(run_dir)
        rid = os.path.basename(run_dir)
        scenario = metrics.get("scenario", "?")
        if ok:
            n_ok += 1
            lines.append("[OK] %s scenario=%s original=%s simplified=%s max_dev=%s cross_track_p95=%s"
                         % (rid, scenario,
                            metrics.get("original_points", "?"),
                            metrics.get("simplified_points", "?"),
                            metrics.get("max_deviation_m", "?"),
                            metrics.get("cross_track_p95", "?")))
        else:
            lines.append("[FAIL] %s scenario=%s" % (rid, scenario))
            for p in problems:
                lines.append("        - %s" % p)

    lines.append("=" * 60)
    lines.append("total runs: %d, ok: %d, fail: %d" % (len(runs), n_ok, len(runs) - n_ok))

    report = "\n".join(lines)
    with open(args.out, "w") as f:
        f.write(report + "\n")
    print(report)
    print("wrote report -> %s" % args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
