# Version5 STM32 trigger sync implementation

## Modified files

- `catkin_ws/src/mvs_ros_driver/src/uvc_driver.cpp`
- `catkin_ws/src/mvs_ros_driver/config/uvc_camera.yaml`
- `catkin_ws/src/FAST-LIVO2/src/LIVMapper.cpp`
- `catkin_ws/src/FAST-LIVO2/include/LIVMapper.h`

## Removed from the camera timestamp main path

- `v4l2_buffer.timestamp`
- CLOCK_MONOTONIC to CLOCK_REALTIME conversion
- V4L2 to MID360 clock-domain mapping
- nearest-trigger selection based on V4L2 time
- MID360 10 Hz point-cloud period as a substitute for camera trigger time

V4L2 is now only used to dequeue and decode image data.

## New camera timestamp rule

The normal camera timestamp source is `/home/nuc/stm32_trigger_time`, written
by an STM32 trigger bridge in the same GPRMC/PPS hardware time domain used by
MID360.

```text
camera_time = stm32_trigger_time + 4.760 ms
            = stm32_trigger_time + 1.760 ms + 3.000 ms
```

`1.760 ms` is trigger-to-exposure-start delay. `3.000 ms` is the center of the
6 ms exposure window.

The existing safety fallback is retained: if STM32 trigger time is missing,
stale, or the selected source is unsupported, the driver uses
`ros::Time::now()` and prints a warning. A valid synchronization test should
show `fallback=0`.

## Frame and trigger matching

The driver reads the camera V4L2 buffer sequence only as a frame counter, not
as a time source. It compares:

- camera `buf.sequence` delta
- STM32 trigger shared-memory sequence delta

If a camera frame is dropped, both deltas should stay consistent. If the deltas
diverge, the log prints a trigger/frame sequence mismatch warning, because a
100 ms frame association error may have occurred.

## PPS boundary

For logging, the code computes:

```text
pps_time = floor(trigger_time / 1s) * 1s
trigger_index = (trigger_time - pps_time) / 100 ms
```

The expected trigger index is 0..9. Crossing from k=9 to k=0 should increase
the PPS second cleanly with no repeated or decreasing camera timestamp.

## FAST-LIVO backend

The backend synchronization logic is unchanged. Version5 only logs the latest
LiDAR begin/end time seen by FAST-LIVO when an image enters the image buffer:

```text
frame_seq
image.header.stamp
lidar_begin_time
lidar_end_time
```

FAST-LIVO still uses LiDAR data before the current camera time for current
fusion, and keeps later LiDAR points in the buffer for the next step.

## STM32 timer check

The firmware under `/home/nuc/LIV_handhold/stm32_timersync-open` initializes:

- `TIM2_PWM_Init(999,7199)` for 10 Hz camera trigger on PA1
- `TIM3_PWM_Init(9999,7199)` for 1 Hz PPS on PB5

In `HARDWARE/TIMER/timer.c`, the TIM3 update interrupt sets:

```c
TIM2->CNT = TIM2->ARR / 2;
```

So the firmware-level phase relation between 1 Hz PPS and 10 Hz camera trigger
is fixed. Physical edge polarity and cable delay still require oscilloscope or
STM32 trigger-event output verification.

## Test commands

```bash
source /home/nuc/versions/version5/ros_env.sh
START_RVIZ=0 SUDO_PASSWORD='***' bash /home/nuc/versions/version5/start_slam1

rostopic hz /uvc_camera/image_sync
rostopic echo -n 5 /uvc_camera/image_sync/header

tail -f /home/nuc/versions/version5/logs/camera_driver.log
tail -f /home/nuc/versions/version5/logs/fast_livo_mapping.log
```

Pass conditions:

- camera driver log shows `source=stm32_trigger_shared`
- `fallback=0` during valid data collection
- camera timestamp period is about 100 ms
- no non-monotonic timestamp warnings
- no repeated camera `header.stamp`
- trigger index cycles 0..9 across PPS boundaries
- FAST-LIVO log shows image time falling into the intended LiDAR begin/end
  sequence without 100 ms jumps
