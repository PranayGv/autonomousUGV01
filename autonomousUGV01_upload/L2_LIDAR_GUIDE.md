# Using the Unitree L2 LiDAR — Mapping & IMU Guide

A practical guide to getting **good 3D maps** and **clean IMU data** from the
Unitree 4D LiDAR L2 on this setup (Ubuntu 26.04 + ROS 2 Lyrical, driver in
`unilidar_sdk2/`, mapping via Point-LIO). Everything here is verified working.

---

## 1. What the L2 gives you

| Stream | Topic | Type | Rate | Frame |
|--------|-------|------|------|-------|
| Point cloud | `/unilidar/cloud` | `sensor_msgs/PointCloud2` | ~12 Hz | `unilidar_lidar` |
| IMU | `/unilidar/imu` | `sensor_msgs/Imu` | **250 Hz** | `unilidar_imu` |

**Point fields:** `x, y, z` (m), `intensity`, `ring` (uint16, 0–17), `time`
(float32, **seconds**, per-point offset within the frame, 0 → ~0.083 s).
~5000 points/frame, 18 scan lines.

**IMU fields & units** (important — these are what make SLAM work):
- `linear_acceleration` — **m/s²** (magnitude ≈ 9.8 at rest; z ≈ +9.8 upright)
- `angular_velocity` — **rad/s** (≈ 0 at rest)
- `orientation` — quaternion from the L2's onboard AHRS (usable, but Point-LIO
  uses the raw gyro/accel, not this)

### Coordinate frame (right-handed)
- **+X = front** (opposite the cable outlet) → this is the robot's forward, red in RViz
- **+Y = left**, **+Z = up**
- IMU frame is parallel to the LiDAR frame, offset by
  `[-0.007698, -0.014655, 0.00667] m` (IMU origin in the LiDAR frame).

---

## 2. Quick start

```bash
cd ~/Documents/Projects/sentrix/slamnav
./lidar_setup_ethernet.sh     # host 192.168.1.2 <-> L2 192.168.1.62 (sudo; once per boot)
./run_point_lio.sh            # driver + Point-LIO + RViz (live map)
```
Hold the LiDAR **still ~5 s** (IMU init → "IMU Initializing: 100%"), then move.

Raw cloud only (no SLAM): `./lidar_run.sh`

---

## 3. Reading the point cloud

```bash
source /opt/ros/lyrical/setup.bash
ros2 topic hz   /unilidar/cloud          # should be ~12 Hz
ros2 topic echo /unilidar/cloud --once   # header + fields
```
In RViz: **PointCloud2** display on `/unilidar/cloud`, Fixed Frame `unilidar_lidar`
(raw) or `camera_init` (mapped, via Point-LIO).

**Tips**
- Colour by `intensity` to see reflectivity, or by `Z` (AxisColor) for height.
- The **rear 120° wedge is filtered out** in the driver (`rear_blind_deg:=120`)
  to hide wires/clutter behind the sensor. Set `rear_blind_deg:=0` to see all 360°.

---

## 4. Reading the IMU

```bash
ros2 topic hz   /unilidar/imu            # should be ~250 Hz
ros2 topic echo /unilidar/imu --once
```

**Verify it's healthy** (do this stationary, on a level surface):
```bash
python3 - <<'PY'
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import Imu
rclpy.init(); n=Node("imu_chk"); s={}
def cb(m):
    a=m.linear_acceleration; g=m.angular_velocity
    s['acc']=math.sqrt(a.x**2+a.y**2+a.z**2); s['gyro']=math.sqrt(g.x**2+g.y**2+g.z**2); rclpy.shutdown()
n.create_subscription(Imu,"/unilidar/imu",cb,50)
rclpy.spin(n)
print(f"|accel| = {s['acc']:.2f} m/s^2  (want ~9.8)")
print(f"|gyro|  = {s['gyro']:.3f} rad/s (want ~0 at rest)")
PY
```
Good IMU: **|accel| ≈ 9.8**, **|gyro| ≈ 0** at rest. If `|accel|` were ≈ 1.0 the
units would be *g* (they aren't here — the L2 reports m/s², so `acc_norm: 9.81`).

**IMU init / bias:** keep the sensor **completely still for the first ~5 s** of
any run — Point-LIO estimates the gyro/accel bias then. Moving during init
corrupts the estimate.

---

## 5. Getting a GOOD map

The mapping quality depends far more on **how you move** than on tuning:

1. **Init still, then translate.** Hold still 5 s, then **walk it through** the
   space. Translation (moving through) builds the map; spinning in place does not.
2. **Move smoothly and moderately.** Slow, steady motion. Avoid fast jerks and
   rapid pure-yaw — those are the hardest for LIO to track.
3. **Keep structure in view.** Point at walls, furniture, corners — flat surfaces
   and edges are what scan-matching locks onto. Big empty/open spaces (beams
   flying to infinity) give it little to hold, and yaw can drift.
4. **Overlap your path.** Revisit areas and close loops so the map stays consistent.
5. **Watch the live map** (RViz `/cloud_registered`, `camera_init` frame). It's a
   rolling 10 s window by default; raise `Decay Time` (or the value in
   `rviz_cfg/loam_livox.rviz`) to keep more of it on screen.

**Save the finished map** (auto-written on graceful Ctrl-C):
```bash
# after Ctrl-C of ./run_point_lio.sh (wait ~3 s):
cp "$(ls -t point_lio_ws/src/point_lio_ros2/PCD/*.pcd | head -1)" ~/slam_map_$(date +%F_%H%M).pcd
pcl_viewer ~/slam_map_*.pcd     # or open in CloudCompare
```

**Sanity-check odometry** any time with `./test_odom.sh` (hold still → walk 2 m →
rotate 90°). Healthy = ~0 drift when still, distance/heading track the real motion.

---

## 6. Critical settings (don't change these)

These are the non-obvious things that make it work — all already set in the scripts:

- **`use_system_timestamp:=false`** — the cloud and IMU **must share the L2's
  device clock**. With `true` (host wall-clock) the cloud and IMU land on
  different clocks and Point-LIO odometry diverges (yaw spins while stationary).
  This is the single most important setting for mapping.
- **`startLidarRotation()`** — the driver must start rotation or the L2 stays in
  standby (answers ping, emits no cloud). Patched into `unitree_lidar_ros2.h`.
- **Persistent host IP** — set via a NetworkManager profile (`lidar_setup_ethernet.sh`),
  not `ip addr add`, which NetworkManager flushes on link events (→ `bind udp
  port failed`).
- **Point-LIO L2 config** (`config/unilidar_l2.yaml`): `lidar_type: 5`,
  `scan_line: 18`, `timestamp_unit: 0` (seconds), `acc_norm: 9.81`,
  `imu_time_inte: 0.004` (250 Hz), `extrinsic_T: [0.007698, 0.014655, -0.00667]`,
  `extrinsic_R: identity`.

---

## 7. Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| Ping OK but no `/unilidar/cloud` | LiDAR in standby → `startLidarRotation()` (already patched); check `ros2 topic hz /unilidar/cloud`. |
| `bind udp port failed` / "Unilidar is not initialized" | Host IP got flushed (run `./lidar_setup_ethernet.sh`) **or** a stale driver holds port 6201 (`pkill -9 -f unitree_lidar_ros2_node`). |
| Map = egocentric "sphere of spokes", yaw spins while still | `use_system_timestamp` was `true` → set `false`. (Odometry diverging.) |
| Map looks like one scan, doesn't accumulate | You're stationary or only rotating — **translate** through the space. RViz decay too short. |
| `|accel|` ≈ 1 not 9.8 | IMU in *g*; set `acc_norm: 1.0` (not our case — L2 is m/s²). |
| L2 not reachable at all | Power + Ethernet into `enp57s0`; L2 in Ethernet mode. Serial fallback: `./lidar_run.sh serial` (needs `dialout` group). |

---

## 8. Reference — the driver command

```bash
ros2 run unitree_lidar_ros2 unitree_lidar_ros2_node --ros-args \
  -p initialize_type:=2 -p work_mode:=0 \
  -p lidar_ip:=192.168.1.62 -p lidar_port:=6101 \
  -p local_ip:=192.168.1.2  -p local_port:=6201 \
  -p cloud_topic:=unilidar/cloud -p cloud_frame:=unilidar_lidar \
  -p imu_topic:=unilidar/imu     -p imu_frame:=unilidar_imu \
  -p cloud_scan_num:=18 -p rear_blind_deg:=120.0 \
  -p use_system_timestamp:=false
```
`initialize_type`: 2 = Ethernet/UDP, 1 = serial (`/dev/ttyACM0` @ 4000000).
`rear_blind_deg`: rear azimuth wedge to drop (0 = full 360°).

---

*See also: `COMMANDS.md` (all robot commands), `LIDAR_README.md` (driver build notes).*
