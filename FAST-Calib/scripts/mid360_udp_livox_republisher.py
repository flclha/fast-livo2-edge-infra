#!/usr/bin/env python3
"""Republish MID360 UDP point packets as livox_ros_driver2/CustomMsg.

This helper is for calibration capture when the MID360 is visibly sending UDP
point packets, but livox_ros_driver2 does not expose /livox/lidar in ROS1.
It parses MID360 Cartesian high-resolution packets sent to host port 56301.
"""

import argparse
import socket
import struct
import sys
import time

import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint

HEADER_LEN = 36
POINT_SIZE = 14
MID360_LINES = 4
DATA_TYPE_CARTESIAN_HIGH = 1


def parse_packet(data):
    if len(data) < HEADER_LEN:
        return None

    packet_len = struct.unpack_from('<H', data, 1)[0]
    time_interval = struct.unpack_from('<H', data, 3)[0]
    dot_num = struct.unpack_from('<H', data, 5)[0]
    data_type = data[10]

    if data_type != DATA_TYPE_CARTESIAN_HIGH or dot_num == 0:
        return None

    need = HEADER_LEN + dot_num * POINT_SIZE
    if len(data) < need or packet_len != len(data):
        return None

    # Livox SDK2 encodes high-resolution Cartesian points as millimetres.
    point_interval_ns = int((time_interval * 100) / dot_num) if dot_num else 0
    points = []
    pos = HEADER_LEN
    for i in range(dot_num):
        x_mm, y_mm, z_mm, reflectivity, tag = struct.unpack_from('<iiiBB', data, pos)
        pos += POINT_SIZE

        pt = CustomPoint()
        pt.offset_time = i * point_interval_ns
        pt.x = x_mm / 1000.0
        pt.y = y_mm / 1000.0
        pt.z = z_mm / 1000.0
        pt.reflectivity = reflectivity
        pt.tag = tag
        pt.line = i % MID360_LINES
        points.append(pt)

    return points


def main():
    parser = argparse.ArgumentParser(description='Publish /livox/lidar from MID360 UDP packets')
    parser.add_argument('--bind-ip', default='192.168.1.5')
    parser.add_argument('--port', type=int, default=56301)
    parser.add_argument('--topic', default='/livox/lidar')
    parser.add_argument('--frame-id', default='livox_frame')
    parser.add_argument('--publish-hz', type=float, default=10.0)
    parser.add_argument('--lidar-id', type=int, default=0)
    args = parser.parse_args()

    rospy.init_node('mid360_udp_livox_republisher', anonymous=False)
    pub = rospy.Publisher(args.topic, CustomMsg, queue_size=16)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind_ip, args.port))
    sock.settimeout(0.2)

    rospy.loginfo('Listening for MID360 UDP on %s:%d, publishing %s', args.bind_ip, args.port, args.topic)
    period = 1.0 / args.publish_hz if args.publish_hz > 0 else 0.1
    next_pub = time.monotonic() + period
    msg_points = []
    packet_count = 0

    while not rospy.is_shutdown():
        try:
            data, _addr = sock.recvfrom(2048)
        except socket.timeout:
            continue

        points = parse_packet(data)
        if not points:
            continue

        packet_count += 1
        if not msg_points:
            batch_started_ns = rospy.Time.now().to_nsec()
            batch_started_mono = time.monotonic()

        elapsed_ns = int((time.monotonic() - batch_started_mono) * 1e9)
        for p in points:
            p.offset_time = (elapsed_ns + int(p.offset_time)) & 0xFFFFFFFF
        msg_points.extend(points)

        now = time.monotonic()
        if now >= next_pub and msg_points:
            stamp = rospy.Time.now()
            msg = CustomMsg()
            msg.header.stamp = stamp
            msg.header.frame_id = args.frame_id
            msg.timebase = int(stamp.to_nsec())
            msg.point_num = len(msg_points)
            msg.lidar_id = args.lidar_id & 0xFF
            msg.rsvd = [0, 0, 0]
            msg.points = msg_points
            pub.publish(msg)

            msg_points = []
            next_pub = now + period
            if packet_count % 100 == 0:
                rospy.loginfo('Republished %d UDP packets', packet_count)

    return 0


if __name__ == '__main__':
    sys.exit(main())
