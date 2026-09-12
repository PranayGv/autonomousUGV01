#!/usr/bin/env bash
#
# run_point_lio.sh  -- real-time 3D mapping with the Unitree L2 (LiDAR + IMU).
#
# Starts the L2 driver (Ethernet/UDP), then Point-LIO, which fuses the point
# cloud and the built-in IMU into live odometry + an incrementally-built map,
# shown in RViz.
#
# Prereq: ./lidar_setup_ethernet.sh  (host must have 192.168.1.2 to reach the L2)
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DISTRO=lyrical
LIDAR_OVERLAY="$SCRIPT_DIR/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash"
PLIO_OVERLAY="$SCRIPT_DIR/point_lio_ws/install/setup.bash"

# ROS setup.bash trips `set -u`; disable nounset only while sourcing.
set +u
source /opt/ros/$DISTRO/setup.bash
[ -f "$LIDAR_OVERLAY" ] && source "$LIDAR_OVERLAY" || { echo "ERR: L2 driver not built."; exit 1; }
[ -f "$PLIO_OVERLAY" ]  && source "$PLIO_OVERLAY"  || { echo "ERR: point_lio not built."; exit 1; }
set -u

# Host must be on the L2 subnet.
if ! ip addr show | grep -q "192.168.1.2"; then
  echo "ERR: host IP 192.168.1.2 not set. Run ./lidar_setup_ethernet.sh first."
  exit 1
fi

# Clear any stale nodes holding the UDP port / topics.
pkill -9 -f unitree_lidar_ros2_node 2>/dev/null || true
pkill -9 -f pointlio_mapping       2>/dev/null || true
sleep 1

cleanup(){
  echo; echo "==> shutting down..."
  kill "${DRV:-}" 2>/dev/null || true
  pkill -9 -f unitree_lidar_ros2_node 2>/dev/null || true
  pkill -9 -f pointlio_mapping       2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "==> [1/2] Starting Unitree L2 driver (Ethernet/UDP)..."
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:=2 -p work_mode:=0 \
  -p lidar_ip:=192.168.1.62 -p lidar_port:=6101 \
  -p local_ip:=192.168.1.2  -p local_port:=6201 \
  -p cloud_topic:=unilidar/cloud -p cloud_frame:=unilidar_lidar \
  -p imu_topic:=unilidar/imu     -p imu_frame:=unilidar_imu \
  -p range_min:=0.0 -p range_max:=100.0 -p cloud_scan_num:=18 -p rear_blind_deg:=120.0 \
  -p use_system_timestamp:=false &
DRV=$!
sleep 4

echo "==> verifying LiDAR + IMU data..."
if timeout 6 ros2 topic hz /unilidar/cloud 2>/dev/null | grep -q "average rate"; then
  echo "   /unilidar/cloud OK"
else
  echo "   WARNING: no /unilidar/cloud yet — check the L2 (ping 192.168.1.62) and that it's spinning."
fi

echo
echo "======================================================================"
echo " Launching Point-LIO. KEEP THE LiDAR STILL for the first ~5 seconds"
echo " so the IMU can initialize. Then move it around to build the map."
echo " Ctrl-C stops everything."
echo "======================================================================"
echo
echo "==> [2/2] Point-LIO mapping + RViz..."
ros2 launch point_lio mapping_unilidar_l2.launch.py
