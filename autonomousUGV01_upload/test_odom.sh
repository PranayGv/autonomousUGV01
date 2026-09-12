#!/usr/bin/env bash
# test_odom.sh -- decisive check that Point-LIO ODOMETRY is healthy.
# FOLLOW THE ON-SCREEN PROMPTS (hold still -> walk ~2 m -> rotate ~90 deg).
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

ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:=2 -p work_mode:=0 \
  -p lidar_ip:=192.168.1.62 -p lidar_port:=6101 -p local_ip:=192.168.1.2 -p local_port:=6201 \
  -p cloud_topic:=unilidar/cloud -p cloud_frame:=unilidar_lidar \
  -p imu_topic:=unilidar/imu -p imu_frame:=unilidar_imu \
  -p cloud_scan_num:=18 -p rear_blind_deg:=120.0 -p use_system_timestamp:=false >/tmp/drv.log 2>&1 &
ros2 launch point_lio mapping_unilidar_l2.launch.py rviz:=false >/tmp/plio.log 2>&1 &
echo "starting... waiting 9s for IMU init  (KEEP IT STILL)"
sleep 9

python3 - <<'PY'
import rclpy, time, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
rclpy.init(); n=Node("t")

# auto-detect the odometry topic (nav_msgs/Odometry)
odom_topic=None
for _ in range(20):
    for name,types in n.get_topic_names_and_types():
        if 'nav_msgs/msg/Odometry' in types and name in ('/aft_mapped_to_init','/odom_corrected','/Odometry'):
            odom_topic=name; break
    if odom_topic: break
    time.sleep(0.3)
if not odom_topic:
    print("ERROR: no Odometry topic found. Topics:")
    for nm,ty in n.get_topic_names_and_types(): print("  ",nm,ty)
    raise SystemExit
print("odom topic =", odom_topic)

S={'odom':None,'imu':None}
n.create_subscription(Odometry,odom_topic,lambda m:S.update(odom=m),50)
n.create_subscription(Imu,"/unilidar/imu",lambda m:S.update(imu=m),100)

# wait for first messages
t0=time.time()
while (S['odom'] is None or S['imu'] is None) and time.time()-t0<15:
    rclpy.spin_once(n,timeout_sec=0.1)
if S['odom'] is None: print("ERROR: subscribed but no odom messages arrived"); raise SystemExit

def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
def samp():
    for _ in range(40): rclpy.spin_once(n,timeout_sec=0.03)
    o=S['odom']; i=S['imu']; p=o.pose.pose.position; q=o.pose.pose.orientation; g=i.angular_velocity
    return p.x,p.y,p.z,yaw(q),math.sqrt(g.x**2+g.y**2+g.z**2)

print("\n===== PHASE 1: HOLD PERFECTLY STILL (6s) =====")
x,y,z,yw,gn=samp(); base=(x,y,yw)
for k in range(4):
    x,y,z,yw,gn=samp()
    print(f"  t{k*1.5:>4}s  pos=({x:+.2f},{y:+.2f},{z:+.2f})  yaw={yw:+.1f}  gyro|w|={gn:.3f} rad/s"); time.sleep(1.5)
print(f"  --> STILL drift: {math.hypot(x-base[0],y-base[1]):.2f} m, {abs(yw-base[2]):.1f} deg  (want ~0)")

print("\n===== PHASE 2: WALK FORWARD ~2 METERS, slowly (9s) =====")
sx,sy,syw=x,y,yw
for k in range(6):
    x,y,z,yw,gn=samp()
    print(f"  t{k*1.5:>4}s  pos=({x:+.2f},{y:+.2f},{z:+.2f})  yaw={yw:+.1f}  gyro|w|={gn:.3f}"); time.sleep(1.5)
print(f"  --> MOVED: {math.hypot(x-sx,y-sy):.2f} m, yaw change {abs(yw-syw):.1f} deg  (want ~2 m, small yaw)")

print("\n===== PHASE 3: ROTATE ~90 deg LEFT in place, slowly (6s) =====")
syw=yw; mg=0
for k in range(4):
    x,y,z,yw,gn=samp(); mg=max(mg,gn)
    print(f"  t{k*1.5:>4}s  pos=({x:+.2f},{y:+.2f})  yaw={yw:+.1f}  gyro|w|={gn:.3f}"); time.sleep(1.5)
print(f"  --> YAW change {yw-syw:+.1f} deg (want ~+90), peak gyro {mg:.2f} rad/s (slow 90deg turn ~1 rad/s)")
n.destroy_node(); rclpy.shutdown()
PY
echo ""; echo "### Point-LIO log tail ###"; tail -6 /tmp/plio.log
