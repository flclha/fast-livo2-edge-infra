# Version5 time synchronization notes

Version5 keeps the version4 workspace structure, but changes the camera
timestamp strategy to `stm32_trigger_shared`.

The camera driver no longer uses V4L2, ROS receive time, callback time, USB
buffer time, or MID360 10 Hz point-cloud period to calculate the normal camera
timestamp. V4L2 is image transport only.

The published camera `header.stamp` is read from the STM32 trigger shared time
source and then shifted to exposure center:

```text
STM32 10 Hz camera trigger time
+ 1.760 ms trigger-to-exposure-start delay
+ 3.000 ms exposure-center offset for 6 ms exposure
+ TimestampOffsetUs manual trim
```

Current parameters:

```text
TimestampSource: stm32_trigger_shared
Stm32TriggerPath: /home/nuc/stm32_trigger_time
Stm32TriggerPeriodUs: 100000
Stm32TriggerToExposureCenterUs: 4760
TimestampOffsetUs: 0
Exposure: 156
```

This assumes the STM32 10 Hz camera trigger phase is fixed in the same
GPRMC/PPS hardware time axis used by MID360. It does not use a LiDAR packet
timestamp as the camera trigger timestamp directly.

If `/home/nuc/stm32_trigger_time` is unavailable or stale, the driver keeps the
existing safety behavior and falls back to `ros::Time::now()` with a warning.
That fallback is only for fault tolerance and should be zero during valid
version5 synchronization tests.

## 2026-08-31 fallback clock-domain fix

A short START_RVIZ=0 startup test showed that /home/nuc/stm32_trigger_time is not present, so the camera driver cannot yet use a real STM32 10 Hz trigger timestamp. To avoid putting Camera in the NUC wall-clock domain while LiDAR/IMU are in the MID360 hardware clock domain, the fallback path now uses ros::Time::now() corrected by /home/nuc/mid360_clock_offset. This is only a compatibility fallback: the final STM32-trigger mode is valid only when the trigger bridge writes /home/nuc/stm32_trigger_time and the camera log shows hardware>0 and fallback=0.

## 2026-08-31 STM32 trigger bridge

Added /home/nuc/versions/version5/tools/stm32_trigger_bridge.py and wired it into /home/nuc/versions/version5/start_slam1. The bridge reads STM32 USART GPRMC at 9600 baud, expands each PPS second into ten 10 Hz trigger timestamps, and writes them with the existing seqlock layout to /home/nuc/stm32_trigger_time. Startup now waits for this file before starting the camera. If no STM32 serial device is present, version5 fails explicitly at the bridge stage instead of silently using an invalid synchronization mode.

## 2026-08-31 no-USB STM32 time-axis mode

Because the NUC cannot receive data from the STM32 over USB/serial, version5 now uses TimestampSource=mid360_trigger_grid. In this mode, V4L2 remains image transport only. The camera driver reads /home/nuc/timeshare, whose timestamp is produced from MID360 packet hardware time after STM32 PPS/GPRMC synchronization, and uses that MID360/STM32 hardware time domain as the common time axis. Each camera frame is assigned to a 100 ms trigger grid using the UVC frame sequence; image.header.stamp is trigger_time + 4.760 ms.

Startup no longer requires /dev/ttyUSB*, /dev/ttyACM*, or /home/nuc/stm32_trigger_time. The serial bridge can still be enabled later with USE_STM32_SERIAL_BRIDGE=1 if a true STM32 serial data path becomes available.

Validation on 2026-08-31 with START_RVIZ=0 succeeded: FAST-LIVO2 started, mapping ran, PCD output was saved, and the camera log reported source=mid360_trigger_grid, hardware>0, fallback=0, period_ms=100.000, trigger_index cycling 0..9 across PPS-second boundaries.
