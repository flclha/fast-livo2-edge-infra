#!/usr/bin/env bash
set -euo pipefail
src_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
stamp=$(date +%Y%m%d_%H%M%S)
guard=/home/nuc/backup/before_restore_version3_$stamp
echo "Restoring version3 from: $src_dir"
echo "Guard backup: $guard"
mkdir -p "$guard/home/nuc/Desktop" "$guard/home/nuc/catkin_ws/src" "$guard/home/nuc/ws_livox/src" "$guard/etc/gdm3" "$guard/etc/udev/rules.d"
cp -a /home/nuc/Desktop/start_slam1 /home/nuc/Desktop/stop_slam "$guard/home/nuc/Desktop/" 2>/dev/null || true
for d in FAST-LIVO2 mvs_ros_driver FAST-Calib; do [ -d "/home/nuc/catkin_ws/src/$d" ] && cp -a "/home/nuc/catkin_ws/src/$d" "$guard/home/nuc/catkin_ws/src/" || true; done
[ -d /home/nuc/ws_livox/src/livox_ros_driver2 ] && cp -a /home/nuc/ws_livox/src/livox_ros_driver2 "$guard/home/nuc/ws_livox/src/" || true
[ -f /etc/gdm3/custom.conf ] && sudo cp -a /etc/gdm3/custom.conf "$guard/etc/gdm3/" || true
for f in /etc/udev/rules.d/99-fast-livo-camera.rules /etc/udev/rules.d/*livox* /etc/udev/rules.d/*camera*; do [ -e "$f" ] && sudo cp -a "$f" "$guard/etc/udev/rules.d/" || true; done
bash /home/nuc/Desktop/stop_slam >/tmp/stop_before_restore_version3.log 2>&1 || true
cp -a "$src_dir/home/nuc/Desktop/start_slam1" /home/nuc/Desktop/start_slam1
cp -a "$src_dir/home/nuc/Desktop/stop_slam" /home/nuc/Desktop/stop_slam
mkdir -p /home/nuc/catkin_ws/src /home/nuc/ws_livox/src
rm -rf /home/nuc/catkin_ws/src/FAST-LIVO2 /home/nuc/catkin_ws/src/mvs_ros_driver /home/nuc/catkin_ws/src/FAST-Calib /home/nuc/ws_livox/src/livox_ros_driver2
cp -a "$src_dir/home/nuc/catkin_ws/src/FAST-LIVO2" /home/nuc/catkin_ws/src/
cp -a "$src_dir/home/nuc/catkin_ws/src/mvs_ros_driver" /home/nuc/catkin_ws/src/
cp -a "$src_dir/home/nuc/catkin_ws/src/FAST-Calib" /home/nuc/catkin_ws/src/
cp -a "$src_dir/home/nuc/ws_livox/src/livox_ros_driver2" /home/nuc/ws_livox/src/
sudo cp -a "$src_dir/etc/gdm3/custom.conf" /etc/gdm3/custom.conf
sudo cp -a "$src_dir/etc/udev/rules.d/"* /etc/udev/rules.d/ 2>/dev/null || true
sudo udevadm control --reload-rules || true
sudo udevadm trigger || true
source /opt/ros/noetic/setup.bash
cd /home/nuc/catkin_ws && catkin_make --pkg mvs_ros_driver fast_livo >/tmp/restore_version3_catkin_ws_build.log 2>&1 || { tail -160 /tmp/restore_version3_catkin_ws_build.log; exit 1; }
cd /home/nuc/ws_livox && catkin_make >/tmp/restore_version3_ws_livox_build.log 2>&1 || { tail -160 /tmp/restore_version3_ws_livox_build.log; exit 1; }
echo "Version3 restore complete. Test with: START_RVIZ=1 bash /home/nuc/Desktop/start_slam1"
