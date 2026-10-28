#!/usr/bin/env python3
"""Inspect existing odometry only to propose fixed motion windows for replay A/B."""

import argparse
import math
import rosbag


def inspect(path):
    with rosbag.Bag(path) as bag:
        start = bag.get_start_time()
        samples = []
        for _, msg, stamp in bag.read_messages(topics=["/aft_mapped_to_init"]):
            p = msg.pose.pose.position
            samples.append((stamp.to_sec() - start, p.x, p.y, p.z))
    if len(samples) < 2:
        print(f"{path}|NO_ODOM")
        return

    duration = samples[-1][0]
    bins = []
    for sec in range(int(math.ceil(duration))):
        points = [p for p in samples if sec <= p[0] < sec + 1]
        distance = 0.0
        for i in range(1, len(points)):
            distance += math.hypot(points[i][1] - points[i - 1][1],
                                   points[i][2] - points[i - 1][2])
        bins.append(distance)

    # A cart is considered moving above 2 cm/s. Fill gaps <=2 s, which usually
    # correspond to brief stops at grid lines rather than a new experiment.
    active = [v >= 0.02 for v in bins]
    for i in range(1, len(active) - 1):
        if not active[i] and active[i - 1] and active[i + 1]:
            active[i] = True
    for i in range(1, len(active) - 2):
        if (not active[i] and not active[i + 1] and
                active[i - 1] and active[i + 2]):
            active[i] = active[i + 1] = True

    indices = [i for i, value in enumerate(active) if value]
    if indices:
        proposed_start = max(0, indices[0] - 1)
        proposed_stop = min(int(math.ceil(duration)), indices[-1] + 2)
    else:
        proposed_start, proposed_stop = 0, int(math.ceil(duration))

    raw_xy = sum(math.hypot(samples[i][1] - samples[i - 1][1],
                            samples[i][2] - samples[i - 1][2])
                 for i in range(1, len(samples)))
    end_xy = math.hypot(samples[-1][1] - samples[0][1],
                        samples[-1][2] - samples[0][2])
    print(f"{path}|duration={duration:.3f}|odom={len(samples)}|"
          f"raw_xy={raw_xy:.3f}|end_xy={end_xy:.3f}|"
          f"window={proposed_start}:{proposed_stop}")
    print("  bins_m=" + " ".join(f"{i}:{v:.3f}" for i, v in enumerate(bins)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+")
    args = parser.parse_args()
    for path in args.bags:
        inspect(path)


if __name__ == "__main__":
    main()
