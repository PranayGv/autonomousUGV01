#!/usr/bin/env bash
#
# run_nav2.sh -- full autonomous stack for the UGV01 tank + L2 LiDAR + Nav2:
#   L2 driver (Ethernet) -> Point-LIO (SLAM: map + odom camera_init->aft_mapped + RViz)
#   UGV01 WiFi driver (/cmd_vel -> /cmd)   |   pointcloud_to_laserscan (/scan)
#   Nav2 (MPPI + NavFn, costmaps from /scan) -> /cmd_vel   |   goal_relay (/goal_pose -> action)
#
# Networks (simultaneous): L2 on Ethernet 192.168.1.x, UGV01 on its WiFi AP 192.168.4.1.
# Set a goal in RViz with "2D Goal Pose" -> the tank plans + drives there.
# Ctrl-C stops everything (and the robot).
#
set -uo pipefail
SD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_IP="${UGV_IP:-192.168.4.1}"

set +u
source /opt/ros/lyrical/setup.bash
source "$SD/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash"
source "$SD/point_lio_ws/install/setup.bash"
set -u

ip addr show | grep -q "192.168.1.2" || { echo "ERR: L2 host IP missing. Run ./lidar_setup_ethernet.sh"; exit 1; }
ping -c1 -W2 "$UGV_IP" >/dev/null 2>&1 || echo "WARN: UGV01 ($UGV_IP) not reachable — are you on its WiFi AP?"

pkill -9 -f unitree_lidar_ros2_node 2>/dev/null || true
pkill -9 -f pointlio_mapping 2>/dev/null || true
pkill -9 -f ugv01_wifi_driver 2>/dev/null || true
pkill -9 -f "controller_server|planner_server|bt_navigator|behavior_server|lifecycle_manager|pointcloud_to_laserscan|goal_relay" 2>/dev/null || true
sleep 1

stop_robot(){ python3 -c "import urllib.request as u; u.urlopen('http://${UGV_IP}/cmd?inputA=1&inputB=0&inputC=0',timeout=0.5).read()" 2>/dev/null || true; }
PIDS=()
cleanup(){
  echo; echo "==> stopping..."; stop_robot
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 2
  pkill -9 -f "unitree_lidar_ros2_node|pointlio_mapping|ugv01_wifi_driver|controller_server|planner_server|bt_navigator|behavior_server|lifecycle_manager|pointcloud_to_laserscan|goal_relay" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "==> [1/5] L2 driver..."
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:=2 -p work_mode:=0 -p lidar_ip:=192.168.1.62 -p lidar_port:=6101 \
  -p local_ip:=192.168.1.2 -p local_port:=6201 -p cloud_topic:=unilidar/cloud \
  -p cloud_frame:=unilidar_lidar -p imu_topic:=unilidar/imu -p imu_frame:=unilidar_imu \
  -p cloud_scan_num:=18 -p rear_blind_deg:=0.0 -p use_system_timestamp:=false &
PIDS+=($!); sleep 4

echo "==> [2/5] Point-LIO (keep robot STILL ~5s for IMU init)..."
ros2 launch point_lio mapping_unilidar_l2.launch.py &
PIDS+=($!); sleep 8

echo "==> [3/5] UGV01 WiFi driver (cmd_vel -> $UGV_IP)..."
python3 "$SD/ugv/ugv01_wifi_driver.py" --ros-args -p robot_ip:="$UGV_IP" &
PIDS+=($!); sleep 2

echo "==> [4/5] goal relay (RViz goal -> Nav2 action)..."
python3 "$SD/nav2/goal_relay.py" &
PIDS+=($!)

# Rolling mode has no saved map, but Nav2 now plans in the 'map' frame, so make
# 'map' identical to Point-LIO's odom frame. (run_map_nav.sh uses slam_toolbox
# to provide map->camera_init instead of this identity.)
ros2 run tf2_ros static_transform_publisher \
  --frame-id map --child-frame-id camera_init >/dev/null 2>&1 &
PIDS+=($!)

echo "==> [5/5] Nav2 (pointcloud_to_laserscan + costmaps + MPPI/NavFn)..."
ros2 launch "$SD/nav2/nav2_bringup.launch.py" &
PIDS+=($!)

echo
echo "======================================================================"
echo " READY. In RViz: '2D Goal Pose' -> the tank plans a path and drives there."
echo " Watch /scan, /global_costmap/costmap, /plan in RViz. Ctrl-C stops all."
echo "======================================================================"
wait
