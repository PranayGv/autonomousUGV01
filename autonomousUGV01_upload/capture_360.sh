#!/usr/bin/env bash
#
# capture_360.sh [ethernet|serial] [seconds] [out.pcd]
#
# Launches the Unitree L2 driver at FULL horizontal FOV (rear_blind_deg:=0.0),
# accumulates the non-repetitive dome scan for a few seconds into one dense
# cloud, saves it to a .pcd, and prints the azimuth/elevation coverage so you
# can confirm a full 360 deg ring (vs. the cropped SLAM launches).
#
# Prereqs (ethernet): sudo ./lidar_setup_ethernet.sh first (host IP 192.168.1.2).
#
set -uo pipefail
MODE="${1:-ethernet}"
SECS="${2:-6}"
OUT="${3:-cloud360.pcd}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY="$SCRIPT_DIR/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash"

set +u
source /opt/ros/lyrical/setup.bash
[ -f "$OVERLAY" ] && source "$OVERLAY" || { echo "ERR: driver not built"; exit 1; }
set -u

case "$MODE" in
  ethernet)
    ip addr show | grep -q "192.168.1.2" || { echo "ERR: run sudo ./lidar_setup_ethernet.sh first"; exit 1; }
    INIT=2 ;;
  serial)
    [ -e /dev/ttyACM0 ] || { echo "ERR: /dev/ttyACM0 not present"; exit 1; }
    INIT=1 ;;
  *) echo "usage: $0 [ethernet|serial] [seconds] [out.pcd]"; exit 1 ;;
esac

# fresh driver, FULL FOV (rear_blind_deg:=0.0 = keep all 360 deg)
pkill -9 -f unitree_lidar_ros2_node 2>/dev/null && sleep 1 || true
echo "==> starting driver at FULL FOV (rear_blind_deg=0)..."
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:="$INIT" -p work_mode:=0 \
  -p serial_port:=/dev/ttyACM0 -p baudrate:=4000000 \
  -p lidar_ip:=192.168.1.62 -p lidar_port:=6101 \
  -p local_ip:=192.168.1.2  -p local_port:=6201 \
  -p cloud_topic:=unilidar/cloud -p cloud_frame:=unilidar_lidar \
  -p imu_topic:=unilidar/imu   -p imu_frame:=unilidar_imu \
  -p range_min:=0.0 -p range_max:=100.0 -p cloud_scan_num:=18 \
  -p rear_blind_deg:=0.0 -p use_system_timestamp:=true >/tmp/capture360_drv.log 2>&1 &
DRV=$!
trap 'kill $DRV 2>/dev/null' EXIT
sleep 4

echo "==> capturing ${SECS}s into ${OUT}..."
python3 "$SCRIPT_DIR/capture_360.py" --topic /unilidar/cloud --seconds "$SECS" --out "$OUT"
