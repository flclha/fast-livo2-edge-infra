# FAST-LIVO2 100s rosbag package

## Files

- `no_camera_lidar_100s.bag`: 100 秒数据包，已排除摄像头图像和激光/点云话题。
- `export_trajectory_pose.py`: 从 bag 中导出小车轨迹和姿态到 CSV 的脚本。

## Bag information

- Duration: 99 s
- Start time: 2026-06-01 12:08:26.72
- End time: 2026-06-01 12:10:06.70
- Size: 254.7 MB

Included topics:

- `/aft_mapped_to_init`: `nav_msgs/Odometry`，推荐用于小车轨迹和姿态。
- `/mavros/vision_pose/pose`: `geometry_msgs/PoseStamped`，也可用于轨迹和姿态。
- `/path`: `nav_msgs/Path`，已累计的路径消息，可在 RViz 中直接显示。
- `/livox/imu`: `sensor_msgs/Imu`，IMU 原始数据。
- `/tf`, `/rosout`, `/rosout_agg`

## View trajectory in RViz

在一个终端播放 bag：

```bash
rosbag play --clock no_camera_lidar_100s.bag
```

在另一个终端打开 RViz：

```bash
rviz
```

RViz 中可以添加：

- `Odometry`，Topic 选择 `/aft_mapped_to_init`
- `Pose`，Topic 选择 `/mavros/vision_pose/pose`
- `Path`，Topic 选择 `/path`

轨迹优先看 `/path` 或 `/aft_mapped_to_init`，实时姿态优先看 `/aft_mapped_to_init`。

## Export trajectory and attitude to CSV

推荐使用脚本导出，输出包含位置、四元数姿态和欧拉角：

```bash
/usr/bin/python3 export_trajectory_pose.py no_camera_lidar_100s.bag \
  --topic /aft_mapped_to_init \
  --out trajectory_pose.csv
```

输出 CSV 字段：

```text
time_sec,x,y,z,qx,qy,qz,qw,roll_rad,pitch_rad,yaw_rad,roll_deg,pitch_deg,yaw_deg
```

字段含义：

- `x,y,z`: 小车位置，即轨迹点。
- `qx,qy,qz,qw`: 小车姿态四元数。
- `roll,pitch,yaw`: 由四元数换算得到的欧拉角，分别是横滚、俯仰、航向角。

也可以从 MAVROS 位姿话题导出：

```bash
/usr/bin/python3 export_trajectory_pose.py no_camera_lidar_100s.bag \
  --topic /mavros/vision_pose/pose \
  --out mavros_vision_pose.csv
```

如果终端在 Conda 的 `(base)` 环境中，优先使用 `/usr/bin/python3`，避免 Conda Python 找不到 ROS 依赖。

## Export with rostopic only

如果只想快速得到原始 CSV，可以使用：

```bash
rostopic echo -b no_camera_lidar_100s.bag -p /aft_mapped_to_init > aft_mapped_to_init_raw.csv
```

或者：

```bash
rostopic echo -b no_camera_lidar_100s.bag -p /mavros/vision_pose/pose > mavros_vision_pose_raw.csv
```

这种方式会保留 ROS 消息原字段，姿态仍是四元数；如果需要欧拉角，使用 `export_trajectory_pose.py` 更方便。
