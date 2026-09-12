#!/usr/bin/env bash
#
# run_map_nav.sh <slam|localize> [map_name]
#
#   slam                : build a NEW 2D map while navigating (slam_toolbox mapping).
#                         Save it any time with:  python3 map_tools.py save <name>
#                         (or the web UI).  Persistent map + relocalization.
#   localize <map_name> : load maps/<map_name> and localize in it (slam_toolbox
#                         localization). Robot knows where it is on the saved map.
#
# Full stack either way: L2 driver + Point-LIO (odom) + slam_toolbox + UGV01 WiFi
# driver + goal_relay + Nav2 (plans in the 'map' frame). Set goals with RViz
# "2D Goal Pose". In localize mode, seed the pose with "2D Pose Estimate" if the
# robot doesn't start where mapping began.
#
set -uo pipefail
SD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UGV_IP="${UGV_IP:-192.168.4.1}"
MODE="${1:-}"
MAPNAME="${2:-}"
MAPS_DIR="$SD/maps"

case "$MODE" in
  slam) ;;
  localize)
    [ -n "$MAPNAME" ] || { echo "usage: $0 localize <map_name>"; exit 1; }
    [ -f "$MAPS_DIR/$MAPNAME.posegraph" ] || {
      echo "ERR: map '$MAPNAME' not found ($MAPS_DIR/$MAPNAME.posegraph missing)."
      echo "Available:"; ls "$MAPS_DIR"/*.posegraph 2>/dev/null | xargs -n1 basename 2>/dev/null | sed 's/.posegraph//'
      exit 1; } ;;
  *) echo "usage: $0 <slam|localize> [map_name]"; exit 1 ;;
esac

set +u
source /opt/ros/lyrical/setup.bash
source "$SD/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash"
source "$SD/point_lio_ws/install/setup.bash"
set -u

ip addr show | grep -q "192.168.1.2" || { echo "ERR: L2 host IP missing. Run ./lidar_setup_ethernet.sh"; exit 1; }
ping -c1 -W2 "$UGV_IP" >/dev/null 2>&1 || echo "WARN: UGV01 ($UGV_IP) not reachable — are you on its WiFi AP?"

KILL_PAT="unitree_lidar_ros2_node|pointlio_mapping|ugv01_wifi_driver|slam_toolbox|async_slam|localization_slam|controller_server|planner_server|bt_navigator|behavior_server|lifecycle_manager|pointcloud_to_laserscan|goal_relay"
pkill -9 -f "$KILL_PAT" 2>/dev/null || true
sleep 1

stop_robot(){ python3 -c "import urllib.request as u,urllib.parse as p; u.urlopen('http://${UGV_IP}/js?json='+p.quote('{\"T\":1,\"L\":0,\"R\":0}'),timeout=0.5).read()" 2>/dev/null || true; }
PIDS=()
cleanup(){
  echo; echo "==> stopping..."; stop_robot
  for pp in "${PIDS[@]}"; do kill "$pp" 2>/dev/null || true; done
  sleep 2
  pkill -9 -f "$KILL_PAT" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "==> [1/6] L2 driver..."
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:=2 -p work_mode:=0 -p lidar_ip:=192.168.1.62 -p lidar_port:=6101 \
  -p local_ip:=192.168.1.2 -p local_port:=6201 -p cloud_topic:=unilidar/cloud \
  -p cloud_frame:=unilidar_lidar -p imu_topic:=unilidar/imu -p imu_frame:=unilidar_imu \
  -p cloud_scan_num:=18 -p rear_blind_deg:=0.0 -p use_system_timestamp:=false &
PIDS+=($!); sleep 4

echo "==> [2/6] Point-LIO (keep robot STILL ~5s for IMU init)..."
ros2 launch point_lio mapping_unilidar_l2.launch.py &
PIDS+=($!); sleep 8

echo "==> [3/6] UGV01 WiFi driver (cmd_vel -> $UGV_IP)..."
python3 "$SD/ugv/ugv01_wifi_driver.py" --ros-args -p robot_ip:="$UGV_IP" &
PIDS+=($!); sleep 2

echo "==> [4/6] goal relay (RViz goal -> Nav2 action)..."
python3 "$SD/nav2/goal_relay.py" &
PIDS+=($!)

if [ "$MODE" = "slam" ]; then
  echo "==> [5/6] slam_toolbox MAPPING (build a new map)..."
  ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
    --params-file "$SD/nav2/slam_toolbox_mapping.yaml" &
  PIDS+=($!)
  echo "    -> save the map any time:  python3 map_tools.py save <name>   (or the web UI)"
else
  echo "==> [5/6] slam_toolbox LOCALIZATION on map '$MAPNAME'..."
  ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
    --params-file "$SD/nav2/slam_toolbox_localization.yaml" \
    -p map_file_name:="$MAPS_DIR/$MAPNAME" &
  PIDS+=($!)
fi
sleep 3

echo "==> [6/6] Nav2 (pointcloud_to_laserscan + costmaps + MPPI/NavFn)..."
ros2 launch "$SD/nav2/nav2_bringup.launch.py" &
PIDS+=($!)

echo
echo "======================================================================"
if [ "$MODE" = "slam" ]; then
echo " MAPPING + NAV. Drive around (RViz '2D Goal Pose') to build the map."
echo " Save it:   python3 map_tools.py save <name>     (keep this running)"
else
echo " LOCALIZING on '$MAPNAME'. If the robot isn't where mapping started,"
echo " use RViz '2D Pose Estimate' to seed its location, then '2D Goal Pose'."
fi
echo " Ctrl-C stops everything (and the robot)."
echo "======================================================================"
wait
