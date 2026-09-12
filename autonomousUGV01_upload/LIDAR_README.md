# Unitree L2 4D LiDAR — live 3D point cloud in RViz

Verified working on **Ubuntu 26.04 + ROS 2 Lyrical Luth**, LiDAR connected via
**Ethernet** (data) + **USB-C** (power). Shows a live `PointCloud2` in RViz.

## Quick start (every boot / after replugging)

```bash
cd ~/Documents/Projects/sentrix/slamnav

# 1. Network setup (sudo). Assigns host IP 192.168.1.2/24 via a persistent
#    NetworkManager profile and pings the LiDAR at 192.168.1.62.
./lidar_setup_ethernet.sh          # wait for "OK: LiDAR reachable"

# 2. Launch driver + RViz (kills any stale driver, then starts fresh)
./lidar_run.sh
```

RViz opens showing the point cloud on `/unilidar/cloud` (fixed frame
`unilidar_lidar`). Confirm data is flowing in a second terminal:

```bash
source /opt/ros/lyrical/setup.bash
ros2 topic hz /unilidar/cloud      # expect a steady rate, not "no new messages"
```

## Connection facts

| Item | Value |
|------|-------|
| LiDAR IP | `192.168.1.62` (port `6101`) |
| Host IP  | `192.168.1.2/24` on `enp57s0` (local port `6201`) |
| Cloud topic / frame | `/unilidar/cloud` / `unilidar_lidar` |
| IMU topic / frame   | `/unilidar/imu` / `unilidar_imu` |
| Serial fallback | `/dev/ttyACM0` @ `4000000` baud (USB-C), `./lidar_run.sh serial` |

Serial mode needs group access once: `sudo usermod -aG dialout $USER` then log out/in.

## Files

| File | Purpose |
|------|---------|
| `lidar_setup_ethernet.sh` | Persistent host-IP setup + LiDAR ping test (sudo) |
| `lidar_run.sh` | Launch driver + RViz (`ethernet` default, or `serial`) |
| `unilidar_sdk2/` | Unitree driver, cloned + patched to build on Lyrical |

## Why the patches were needed (Lyrical port)

The upstream `unilidar_sdk2` driver targets an older ROS 2 and does **not** work
on Lyrical unmodified. Fixes applied in the cloned copy:

1. **CMake** — `ament_target_dependencies()` was removed in Lyrical; replaced
   with modern namespaced targets (`rclcpp::rclcpp`, `sensor_msgs::sensor_msgs`,
   `tf2_ros::tf2_ros`, `pcl_conversions::pcl_conversions`) in `CMakeLists.txt`.
2. **Headers** — tf2 renamed `.h`→`.hpp`; fixed `Quaternion.hpp` and
   `transform_broadcaster.hpp` in `unitree_lidar_ros2.h`.
3. **No data despite ping OK** — the stock node never called
   `startLidarRotation()`, so the L2 stayed in standby (answers ping, emits no
   cloud). Added it after `setLidarWorkMode()` in `unitree_lidar_ros2.h`.

If you re-clone the driver, re-apply these and rebuild:

```bash
cd unilidar_sdk2/unitree_lidar_ros2 && colcon build
```

## Troubleshooting

| Symptom | Cause / fix |
|---------|-------------|
| `ping FAILS`, host has only `169.254.x` | Host IP flushed by NetworkManager → re-run `./lidar_setup_ethernet.sh` (persistent nmcli profile) |
| `bind udp port failed` / `Unilidar is not initialized` | Host IP `192.168.1.2` missing, **or** a stale driver holds port `6201` → `lidar_run.sh` now clears both |
| Ping OK but no cloud, `rx_packets` low | LiDAR not spinning → `startLidarRotation()` fix (already applied) |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` vs ROS `setup.bash` → sourcing is wrapped in `set +u` |
