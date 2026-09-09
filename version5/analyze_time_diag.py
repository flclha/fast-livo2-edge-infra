#!/usr/bin/env python3
import argparse
import csv
import json
import math
import os
from bisect import bisect_left

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except Exception:
    plt = None


def ns_to_ms(x):
    return float(x) / 1e6


def summary_ms(values):
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {"count": 0}
    return {
        "count": int(arr.size),
        "mean_ms": float(np.mean(arr)),
        "mean_abs_ms": float(np.mean(np.abs(arr))),
        "median_ms": float(np.median(arr)),
        "median_abs_ms": float(np.median(np.abs(arr))),
        "p95_abs_ms": float(np.percentile(np.abs(arr), 95)),
        "max_abs_ms": float(np.max(np.abs(arr))),
        "std_ms": float(np.std(arr)),
    }


def nearest_diffs(target_ns, reference_ns):
    ref = sorted(int(x) for x in reference_ns)
    out = []
    matched = []
    for t in target_ns:
        t = int(t)
        i = bisect_left(ref, t)
        if i <= 0:
            r = ref[0]
        elif i >= len(ref):
            r = ref[-1]
        else:
            before, after = ref[i - 1], ref[i]
            r = before if abs(t - before) <= abs(t - after) else after
        out.append(ns_to_ms(t - r))
        matched.append(r)
    return np.asarray(out), np.asarray(matched)


def load_diag_csv(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({
                "frame_index": int(row["frame_index"]),
                "v4l2_host_realtime_ns": int(row["v4l2_host_realtime_ns"]),
                "host_minus_lidar_ns": int(row["host_minus_lidar_ns"]),
                "camera_header_ns": int(row["camera_header_ns"]),
                "timestamp_offset_us": int(row["timestamp_offset_us"]),
                "v4l2_flags": int(row["v4l2_flags"]),
            })
    return rows


def load_bag_timestamps(bag_path, topics):
    import rosbag
    stamps = {k: [] for k in topics}
    receives = {k: [] for k in topics}
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=list(topics.values())):
            name = [k for k, v in topics.items() if v == topic][0]
            if hasattr(msg, "header"):
                stamps[name].append(int(msg.header.stamp.to_nsec()))
            receives[name].append(int(t.to_nsec()))
    return stamps, receives


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--diag-csv", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--exposure-ms", type=float, default=6.0)
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    topics = {
        "camera": "/uvc_camera/image_sync",
        "lidar": "/livox/lidar",
        "imu": "/livox/imu",
    }
    stamps, receives = load_bag_timestamps(args.bag, topics)
    rows = load_diag_csv(args.diag_csv)
    if stamps["camera"]:
        lo = min(stamps["camera"]) - 1000000
        hi = max(stamps["camera"]) + 1000000
        rows = [r for r in rows if lo <= r["camera_header_ns"] <= hi]
    if not rows:
        raise RuntimeError("No diagnostic camera rows overlap the rosbag camera timestamps")

    cam_header = np.asarray([r["camera_header_ns"] for r in rows], dtype=np.int64)
    offset_us = rows[0]["timestamp_offset_us"] if rows else 0
    v4l2_soe_mid360 = cam_header - int(offset_us * 1000)
    v4l2_center_mid360 = v4l2_soe_mid360 + int(args.exposure_ms * 0.5 * 1e6)

    trigger_grid_ns = np.rint(v4l2_soe_mid360.astype(np.float64) / 1e8).astype(np.int64) * 100000000
    stm32_pred_soe = trigger_grid_ns + 1760000
    stm32_pred_center = trigger_grid_ns + int((1.760 + args.exposure_ms * 0.5) * 1e6)

    rows_out = []
    for i in range(len(rows)):
        rows_out.append({
            "frame_index": rows[i]["frame_index"],
            "v4l2_soe_mid360_ns": int(v4l2_soe_mid360[i]),
            "v4l2_center_mid360_ns": int(v4l2_center_mid360[i]),
            "stm32_trigger_grid_ns": int(trigger_grid_ns[i]),
            "stm32_pred_center_ns": int(stm32_pred_center[i]),
            "v4l2_soe_minus_trigger_ms": ns_to_ms(v4l2_soe_mid360[i] - trigger_grid_ns[i]),
            "v4l2_center_minus_stm32_pred_center_ms": ns_to_ms(v4l2_center_mid360[i] - stm32_pred_center[i]),
        })

    out_csv = os.path.join(args.outdir, "time_axis_frame_deltas.csv")
    with open(out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows_out[0].keys()))
        writer.writeheader()
        writer.writerows(rows_out)

    cam_lidar_ms, _ = nearest_diffs(cam_header, stamps["lidar"])
    lidar_cam_ms, _ = nearest_diffs(stamps["lidar"], cam_header)
    cam_imu_ms, _ = nearest_diffs(cam_header, stamps["imu"])

    result = {
        "input": {
            "bag": args.bag,
            "diag_csv": args.diag_csv,
            "exposure_ms": args.exposure_ms,
            "timestamp_offset_us_in_driver": offset_us,
            "stm32_model": "nearest 10 Hz grid in MID360/GPRMC clock domain; trigger-to-SOE delay 1.760 ms",
        },
        "counts": {k: len(v) for k, v in stamps.items()},
        "v4l2_soe_minus_stm32_trigger_grid": summary_ms([r["v4l2_soe_minus_trigger_ms"] for r in rows_out]),
        "v4l2_center_minus_stm32_pred_center": summary_ms([r["v4l2_center_minus_stm32_pred_center_ms"] for r in rows_out]),
        "camera_header_minus_nearest_lidar_header": summary_ms(cam_lidar_ms),
        "lidar_header_minus_nearest_camera_header": summary_ms(lidar_cam_ms),
        "camera_header_minus_nearest_imu_header": summary_ms(cam_imu_ms),
    }
    out_json = os.path.join(args.outdir, "time_axis_summary.json")
    with open(out_json, "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    if plt is not None:
        elapsed = (cam_header - cam_header[0]) / 1e9
        plt.figure(figsize=(12, 5))
        plt.plot(elapsed, [r["v4l2_soe_minus_trigger_ms"] for r in rows_out], label="V4L2 SOE - STM32 10Hz grid")
        plt.plot(elapsed, [r["v4l2_center_minus_stm32_pred_center_ms"] for r in rows_out], label="V4L2 center - STM32 predicted center")
        plt.axhline(0, color="k", linewidth=0.8)
        plt.xlabel("Elapsed time (s)")
        plt.ylabel("Delta t (ms)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(args.outdir, "v4l2_vs_stm32_predicted_time.png"), dpi=160)

        plt.figure(figsize=(12, 5))
        x2 = (np.asarray(stamps["camera"]) - stamps["camera"][0]) / 1e9
        plt.plot(x2, cam_lidar_ms[:len(x2)], label="Camera header - nearest LiDAR header")
        plt.axhline(0, color="k", linewidth=0.8)
        plt.xlabel("Elapsed time (s)")
        plt.ylabel("Delta t (ms)")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(args.outdir, "camera_lidar_header_alignment.png"), dpi=160)

    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
