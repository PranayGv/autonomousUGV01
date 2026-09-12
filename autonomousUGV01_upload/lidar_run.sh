#!/usr/bin/env bash
#
# lidar_run.sh [ethernet|serial]
# Launches the Unitree L2 driver + RViz to visualize the 3D point cloud.
#   ethernet (default): UDP mode, lidar 192.168.1.62 -> host 192.168.1.2
#                       (run ./lidar_setup_ethernet.sh first)
#   serial            : reads /dev/ttyACM0 @ 4000000 baud over USB-C
#
set -uo pipefail

MODE="${1:-ethernet}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OVERLAY="$SCRIPT_DIR/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash"

# ROS setup.bash references unbound vars; disable nounset just around sourcing.
set +u
source /opt/ros/lyrical/setup.bash
if [ -f "$OVERLAY" ]; then
  source "$OVERLAY"
else
  set -u
  echo "ERR: driver not built. Build it with:"
  echo "  cd $SCRIPT_DIR/unilidar_sdk2/unitree_lidar_ros2 && colcon build"
  exit 1
fi
set -u

RVIZ_CFG="$SCRIPT_DIR/unilidar_sdk2/unitree_lidar_ros2/src/unitree_lidar_ros2/rviz/view.rviz"

case "$MODE" in
  ethernet)
    echo "==> Ethernet/UDP mode (lidar 192.168.1.62 -> host 192.168.1.2)"
    if ! ip addr show | grep -q "192.168.1.2"; then
      echo "ERR: host IP 192.168.1.2 is not configured (NetworkManager may have flushed it)."
      echo "     Run ./lidar_setup_ethernet.sh first, then retry."
      exit 1
    fi
    INIT_TYPE=2
    ;;
  serial)
    echo "==> Serial mode (/dev/ttyACM0 @ 4000000)"
    if [ ! -e /dev/ttyACM0 ]; then echo "ERR: /dev/ttyACM0 not present."; exit 1; fi
    if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
      echo "ERR: you are not in the 'dialout' group, so /dev/ttyACM0 cannot be opened."
      echo "     Fix once with:  sudo usermod -aG dialout \$USER   (then log out/in)"
      exit 1
    fi
    INIT_TYPE=1
    ;;
  *) echo "Usage: $0 [ethernet|serial]"; exit 1 ;;
esac

# Kill any stale driver from a previous run — it holds UDP port 6201 and blocks
# the new node's bind ("bind udp port failed").
if pgrep -f unitree_lidar_ros2_node >/dev/null; then
  echo "==> Killing stale driver process(es) holding the UDP port..."
  pkill -9 -f unitree_lidar_ros2_node 2>/dev/null || true
  sleep 1
fi

echo "==> Starting driver node..."
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:="$INIT_TYPE" \
  -p work_mode:=0 \
  -p serial_port:=/dev/ttyACM0 -p baudrate:=4000000 \
  -p lidar_ip:=192.168.1.62 -p lidar_port:=6101 \
  -p local_ip:=192.168.1.2  -p local_port:=6201 \
  -p cloud_topic:=unilidar/cloud -p cloud_frame:=unilidar_lidar \
  -p imu_topic:=unilidar/imu   -p imu_frame:=unilidar_imu \
  -p range_min:=0.0 -p range_max:=100.0 -p cloud_scan_num:=18 \
  -p use_system_timestamp:=true &
NODE_PID=$!
trap 'kill $NODE_PID $RVIZ_PID 2>/dev/null' INT TERM EXIT

sleep 2
echo "==> Opening RViz (PointCloud2 on /unilidar/cloud, fixed frame unilidar_lidar)..."
rviz2 -d "$RVIZ_CFG" &
RVIZ_PID=$!

echo
echo "If RViz shows no points: in another terminal check data is arriving with"
echo "   ros2 topic hz /unilidar/cloud"
echo "Ctrl-C here stops the driver and RViz."
wait $NODE_PID
