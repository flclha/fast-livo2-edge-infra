#!/usr/bin/python3
import argparse
import csv
import math

import rosbag
from tf.transformations import euler_from_quaternion


def pose_from_msg(msg):
    msg_type = getattr(msg, "_type", "")
    if msg_type == "nav_msgs/Odometry" or hasattr(getattr(msg, "pose", None), "pose"):
        return msg.pose.pose
    if msg_type == "geometry_msgs/PoseStamped" or hasattr(msg, "pose"):
        return msg.pose
    raise TypeError("Unsupported message type: {}".format(msg_type or type(msg).__name__))


def stamp_to_sec(msg, fallback_time):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is not None and stamp.to_sec() > 0:
        return stamp.to_sec()
    return fallback_time.to_sec()


def export_pose_csv(bag_path, topic, out_path):
    rows = 0
    with rosbag.Bag(bag_path, "r") as bag, open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "time_sec",
                "x",
                "y",
                "z",
                "qx",
                "qy",
                "qz",
                "qw",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
            ]
        )

        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            pose = pose_from_msg(msg)
            position = pose.position
            orientation = pose.orientation
            quat = [orientation.x, orientation.y, orientation.z, orientation.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            writer.writerow(
                [
                    "{:.9f}".format(stamp_to_sec(msg, bag_time)),
                    "{:.9f}".format(position.x),
                    "{:.9f}".format(position.y),
                    "{:.9f}".format(position.z),
                    "{:.9f}".format(orientation.x),
                    "{:.9f}".format(orientation.y),
                    "{:.9f}".format(orientation.z),
                    "{:.9f}".format(orientation.w),
                    "{:.9f}".format(roll),
                    "{:.9f}".format(pitch),
                    "{:.9f}".format(yaw),
                    "{:.9f}".format(math.degrees(roll)),
                    "{:.9f}".format(math.degrees(pitch)),
                    "{:.9f}".format(math.degrees(yaw)),
                ]
            )
            rows += 1

    return rows


def main():
    parser = argparse.ArgumentParser(
        description="Export vehicle trajectory and attitude from a ROS bag."
    )
    parser.add_argument("bag", help="Input .bag file")
    parser.add_argument(
        "--topic",
        default="/aft_mapped_to_init",
        help="Pose topic, for example /aft_mapped_to_init or /mavros/vision_pose/pose",
    )
    parser.add_argument(
        "--out",
        default="trajectory_pose.csv",
        help="Output CSV path",
    )
    args = parser.parse_args()

    rows = export_pose_csv(args.bag, args.topic, args.out)
    print("Exported {} rows from {} to {}".format(rows, args.topic, args.out))


if __name__ == "__main__":
    main()
