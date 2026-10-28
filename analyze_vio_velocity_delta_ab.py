#!/usr/bin/env python3
"""Analyze paired offline VIO velocity-delta guard runs.

This script intentionally uses the published final odometry for Z/XY metrics and
the FAST-LIVO innovation CSV for LiDAR residuals.  It never changes a ROS/YAML
parameter.  Motion windows are relative to the original input bag start.
"""

import argparse
import csv
import json
import math
import os
import statistics
import subprocess
from typing import Dict, List, Sequence, Tuple


def percentile(values: Sequence[float], q: float) -> float:
    vals = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not vals:
        return float("nan")
    p = (len(vals) - 1) * q
    lo = int(math.floor(p))
    hi = int(math.ceil(p))
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - p) + vals[hi] * (p - lo)


def mean(values: Sequence[float]) -> float:
    vals = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.fmean(vals) if vals else float("nan")


def read_meta(run_dir: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    with open(os.path.join(run_dir, "run_meta.txt"), encoding="utf-8") as f:
        for line in f:
            if "=" in line:
                key, value = line.rstrip("\n").split("=", 1)
                out[key] = value
    return out


def read_odom(run_dir: str, bag_start: float, window: Tuple[float, float]):
    bag = os.path.join(run_dir, "derived.bag")
    code = (
        "import rosbag,sys\n"
        "b=rosbag.Bag(sys.argv[1])\n"
        "for _,m,t in b.read_messages(topics=['/aft_mapped_to_init']):\n"
        " p=m.pose.pose.position\n"
        " print('%.9f,%.9f,%.12g,%.12g,%.12g' % (t.to_sec(),m.header.stamp.to_sec(),p.x,p.y,p.z))\n"
        "b.close()\n"
    )
    text = subprocess.check_output(["python3", "-c", code, bag], text=True)
    rows = []
    for line in text.splitlines():
        vals = [float(v) for v in line.split(",")]
        rel = vals[0] - bag_start
        if window[0] <= rel <= window[1]:
            rows.append((rel, vals[1], vals[2], vals[3], vals[4]))
    return rows


def lidar_clock_alignment(input_bag: str):
    """Return (first LiDAR hardware/header time, its record time)."""
    code = (
        "import rosbag,sys\n"
        "b=rosbag.Bag(sys.argv[1])\n"
        "for _,m,t in b.read_messages(topics=['/livox/lidar']):\n"
        " print('%.12f,%.12f' % (m.header.stamp.to_sec(),t.to_sec())); break\n"
        "b.close()\n"
    )
    text = subprocess.check_output(["python3", "-c", code, input_bag], text=True).strip()
    return tuple(float(v) for v in text.split(","))


def read_csv(run_dir: str, bag_start: float, input_bag: str, window: Tuple[float, float]):
    path = os.path.join(run_dir, "lio_innovation.csv")
    lidar_header0, lidar_record0 = lidar_clock_alignment(input_bag)
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                # lidar_time is on the MID360 hardware clock, whereas rosbag
                # record time is on the host/system clock.  Align the two at
                # the first LiDAR packet; never subtract unlike time domains.
                rel = (float(row["lidar_time"]) - lidar_header0) + (lidar_record0 - bag_start)
            except (KeyError, ValueError):
                continue
            if window[0] <= rel <= window[1]:
                rows.append(row)
    return rows


def smooth_xy(xy: Sequence[Tuple[float, float]], radius: int = 5):
    smoothed = []
    for i in range(len(xy)):
        lo = max(0, i - radius)
        hi = min(len(xy), i + radius + 1)
        smoothed.append((mean([p[0] for p in xy[lo:hi]]), mean([p[1] for p in xy[lo:hi]])))
    return smoothed


def path_length(xy: Sequence[Tuple[float, float]]) -> float:
    return sum(math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1]) for i in range(1, len(xy)))


def f(row: Dict[str, str], key: str, default=float("nan")) -> float:
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return default


def analyze_run(run_dir: str, start: float, stop: float):
    meta = read_meta(run_dir)
    bag_start = float(meta["bag_start"])
    window = (start, stop)
    odom = read_odom(run_dir, bag_start, window)
    diag = read_csv(run_dir, bag_start, meta["bag"], window)
    if len(odom) < 3 or len(diag) < 3:
        raise RuntimeError(f"insufficient rows in {run_dir}: odom={len(odom)}, diag={len(diag)}")

    z_steps = [abs(odom[i][4] - odom[i - 1][4]) * 1000.0 for i in range(1, len(odom))]
    xy = [(r[2], r[3]) for r in odom]
    raw_len = path_length(xy)
    smooth_len = path_length(smooth_xy(xy))
    xy_roughness = raw_len / smooth_len if smooth_len > 1e-9 else float("nan")
    weighted_residual = [abs(f(r, "weighted_average_residual")) for r in diag]
    raw_prediction = [abs(f(r, "raw_prediction_step_z")) * 1000.0 for r in diag]
    if meta.get("stage") == "prediction_start":
        applied_key = "prediction_start_velocity_delta_guard_applied"
        delta_key = "prediction_start_velocity_delta_normal_before"
    else:
        applied_key = "previous_vio_velocity_delta_guard_applied"
        delta_key = "previous_vio_velocity_delta_normal_before"
    guard_applied = [f(r, applied_key, 0.0) for r in diag]
    delta_before = [abs(f(r, delta_key)) * 1000.0 for r in diag]

    return {
        "run_dir": run_dir,
        "label": meta.get("label", ""),
        "mode": meta.get("mode", ""),
        "trigger": float(meta.get("trigger", "nan")),
        "limit": float(meta.get("limit", "nan")),
        "motion_start_s": start,
        "motion_stop_s": stop,
        "odom_rows": len(odom),
        "diag_rows": len(diag),
        "z_jump_count": sum(v > 10.0 for v in z_steps),
        "z_step_p95_mm": percentile(z_steps, 0.95),
        "z_step_p99_mm": percentile(z_steps, 0.99),
        "z_step_max_mm": max(z_steps),
        "xy_raw_path_m": raw_len,
        "xy_smoothed_path_m": smooth_len,
        "xy_path_roughness": xy_roughness,
        "lidar_weighted_residual_mean": mean(weighted_residual),
        "lidar_weighted_residual_p95": percentile(weighted_residual, 0.95),
        "raw_prediction_p95_mm": percentile(raw_prediction, 0.95),
        "raw_prediction_p99_mm": percentile(raw_prediction, 0.99),
        "raw_prediction_max_mm": max(raw_prediction),
        "guard_applied_count": int(sum(v > 0.5 for v in guard_applied)),
        "vio_delta_normal_p95_mm_s": percentile(delta_before, 0.95),
        "vio_delta_normal_p99_mm_s": percentile(delta_before, 0.99),
        "vio_delta_normal_max_mm_s": max(delta_before),
    }


def aggregate(rows: Sequence[Dict[str, float]]):
    metrics = [
        "z_jump_count", "z_step_p95_mm", "z_step_p99_mm", "z_step_max_mm",
        "xy_path_roughness", "lidar_weighted_residual_mean",
        "lidar_weighted_residual_p95", "raw_prediction_p95_mm",
        "raw_prediction_p99_mm", "raw_prediction_max_mm", "guard_applied_count",
    ]
    return {k: mean([r[k] for r in rows]) for k in metrics}


def pct(new: float, old: float) -> float:
    return (new / old - 1.0) * 100.0 if old else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pair", action="append", nargs=4, metavar=("OFF", "ON", "START", "STOP"), required=True)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    off_rows, on_rows = [], []
    pairs = []
    for off_dir, on_dir, start_s, stop_s in args.pair:
        off = analyze_run(off_dir, float(start_s), float(stop_s))
        on = analyze_run(on_dir, float(start_s), float(stop_s))
        off_rows.append(off)
        on_rows.append(on)
        pairs.append({"off": off, "on": on})

    off_agg = aggregate(off_rows)
    on_agg = aggregate(on_rows)
    changes = {k: pct(on_agg[k], off_agg[k]) for k in off_agg}
    z_keys = ["z_jump_count", "z_step_p95_mm", "z_step_p99_mm", "z_step_max_mm"]
    z_all_improve = all(on_agg[k] < off_agg[k] for k in z_keys)
    xy_ok = changes["xy_path_roughness"] <= 2.0
    residual_ok = changes["lidar_weighted_residual_mean"] <= 2.0
    per_bag_z_all_improve = all(all(p["on"][k] < p["off"][k] for k in z_keys) for p in pairs)
    accepted = z_all_improve and xy_ok and residual_ok and per_bag_z_all_improve
    report = {
        "pairs": pairs,
        "aggregate_off": off_agg,
        "aggregate_on": on_agg,
        "percent_change_on_vs_off": changes,
        "criteria": {
            "aggregate_all_four_z_improved": z_all_improve,
            "each_bag_all_four_z_improved": per_bag_z_all_improve,
            "xy_roughness_worsening_le_2pct": xy_ok,
            "lidar_residual_worsening_le_2pct": residual_ok,
            "accepted": accepted,
        },
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as fobj:
        json.dump(report, fobj, indent=2, ensure_ascii=False)

    print("metric                              OFF mean       ON mean    change")
    print("-------------------------------------------------------------------")
    for key in z_keys + ["xy_path_roughness", "lidar_weighted_residual_mean", "lidar_weighted_residual_p95"]:
        print(f"{key:34s} {off_agg[key]:11.6f} {on_agg[key]:11.6f} {changes[key]:+8.3f}%")
    print("-------------------------------------------------------------------")
    print(json.dumps(report["criteria"], ensure_ascii=False))
    print(f"REPORT={args.output}")


if __name__ == "__main__":
    main()
