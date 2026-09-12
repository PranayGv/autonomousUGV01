#!/usr/bin/env bash
# diag_point_lio.sh -- diagnose why Point-LIO isn't building a map.
# Run with the L2 powered + ./lidar_setup_ethernet.sh already done.
set -uo pipefail
SD="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/lyrical/setup.bash
source "$SD/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash"
source "$SD/point_lio_ws/install/setup.bash"
set -u

pkill -9 -f unitree_lidar_ros2_node 2>/dev/null || true
pkill -9 -f pointlio_mapping 2>/dev/null || true
sleep 1
trap 'pkill -9 -f unitree_lidar_ros2_node 2>/dev/null; pkill -9 -f pointlio_mapping 2>/dev/null' EXIT

echo "### ping L2"; ping -c1 -W2 192.168.1.62 >/dev/null 2>&1 && echo "  reachable" || echo "  NOT reachable"

echo "### starting driver"
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:=2 -p work_mode:=0 \
  -p lidar_ip:=192.168.1.62 -p lidar_port:=6101 -p local_ip:=192.168.1.2 -p local_port:=6201 \
  -p cloud_topic:=unilidar/cloud -p cloud_frame:=unilidar_lidar \
  -p imu_topic:=unilidar/imu -p imu_frame:=unilidar_imu \
  -p cloud_scan_num:=18 -p use_system_timestamp:=true >/tmp/drv.log 2>&1 &
sleep 5

echo "### IMU rate (5s):";   timeout 6 ros2 topic hz /unilidar/imu   2>/dev/null | grep -m1 "average rate" || echo "  NO IMU DATA"
echo "### CLOUD rate (5s):"; timeout 6 ros2 topic hz /unilidar/cloud 2>/dev/null | grep -m1 "average rate" || echo "  NO CLOUD DATA"

echo "### IMU sample (accel should be ~9.8 magnitude at rest):"
timeout 5 ros2 topic echo /unilidar/imu --once 2>/dev/null | grep -A4 "linear_acceleration" | head -5

echo "### cloud PointField layout + per-point time sanity:"
python3 - <<'PY'
import rclpy, sys
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
rclpy.init()
n=Node("diag")
got={}
def cb(m):
    got['fields']=[(f.name,f.offset,f.datatype) for f in m.fields]
    pts=list(pc2.read_points(m, field_names=None, skip_nans=True))
    got['npts']=len(pts)
    # print first few 'time' values if present
    names=[f.name for f in m.fields]
    if 'time' in names:
        ti=names.index('time')
        got['time_samples']=[float(pts[i][ti]) for i in range(0,min(len(pts),2000),400)]
    n.destroy_node(); rclpy.shutdown()
s=n.create_subscription(PointCloud2,"/unilidar/cloud",cb,10)
import time
t=time.time()
while rclpy.ok() and time.time()-t<5: rclpy.spin_once(n,timeout_sec=0.2)
print("  fields:",got.get('fields'))
print("  n_points:",got.get('npts'))
print("  time samples:",got.get('time_samples'))
PY

echo "### starting Point-LIO for 15s (KEEP STILL 5s, then MOVE it):"
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false >/tmp/plio.log 2>&1 &
sleep 6
echo "--- /Odometry pose at t=6s, 9s, 12s (should CHANGE if you move it) ---"
for t in 1 2 3; do
  timeout 4 ros2 topic echo /aft_mapped_to_init --once 2>/dev/null | grep -A3 "position" | head -4 \
    || timeout 4 ros2 topic echo /Odometry --once 2>/dev/null | grep -A3 "position" | head -4 \
    || echo "  (no odometry topic)"
  sleep 3
done
echo "### Point-LIO log (last 25 lines) ###"; tail -25 /tmp/plio.log
echo "### driver log (last 8) ###"; tail -8 /tmp/drv.log
echo "### DONE"
