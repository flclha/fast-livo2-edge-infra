This version keeps camera timestamps as shutter timestamps when TriggerEnable=1.
- mvs_ros_driver/src/uvc_driver.cpp: no silent ros::Time::now() fallback in trigger mode; validates /home/<user>/timeshare timestamp.
- mvs_ros_driver/src/grab_trigger.cpp: same timestamp policy for MVS trigger driver source.
- FAST-LIVO2/scripts/uvc_image_stamp_relay.py: preserves incoming msg.header.stamp instead of replacing it with relay receive time.
Important: /home/nuc/timeshare must be updated by the STM32/PC-side timestamp writer. If it is stale or missing, the camera driver will skip frames and print ROS_ERROR instead of publishing incorrect receive-time timestamps.

Added after initial timestamp fix:
- FAST-LIVO2/scripts/timeshare_from_livox.py writes ~/timeshare from /livox/lidar header.stamp.
- ROS_FASTLIVO_Tracer/scripts/start_slam1 starts livox_timeshare_writer automatically.
- uvc_image_stamp_relay default min_interval is now 0.0 to avoid limiting images to 1 Hz.
