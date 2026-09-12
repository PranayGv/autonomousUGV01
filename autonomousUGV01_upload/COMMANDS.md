# Sentrix SLAM robot — command reference

Robot = **Waveshare UGV01 tank** (WiFi JSON API) + **Unitree L2** 4D LiDAR.
Odometry + 3D map come from the L2 via **Point-LIO** (the UGV01 has no encoders/IMU
over WiFi); the tank is a pure actuator driven by Nav2 `/cmd_vel`.
All commands run from `~/Documents/Projects/sentrix/slamnav`.

```bash
cd ~/Documents/Projects/sentrix/slamnav
```

## Networks (both at once)
- **L2 LiDAR** on Ethernet — robot `192.168.1.62`, host `192.168.1.2/24` on `enp57s0`.
- **UGV01** is its own WiFi AP — **SSID `UGV` / `12345678`**, robot `http://192.168.4.1`.
  Join the UGV WiFi to drive; the L2 keeps working on Ethernet simultaneously.

## 0. Setup (once per boot)
```bash
sudo ./lidar_setup_ethernet.sh      # gives host 192.168.1.2 to reach the L2
# then join the UGV WiFi AP (SSID UGV / 12345678)
```

## 1. Autonomous navigation (Nav2, live map)
```bash
./run_nav2.sh
```
L2 driver + Point-LIO (odom + 3D map + RViz) + `pointcloud_to_laserscan` (/scan) +
Nav2 (MPPI + NavFn costmaps) + UGV01 WiFi driver + goal_relay.
Hold the robot **still ~5 s** at start (Point-LIO IMU init). Then set a goal in RViz
with **2D Goal Pose** — the tank plans a path and drives there, avoiding obstacles.
`Ctrl-C` stops everything (and the robot, via the driver watchdog).

## 2. Persistent map: build once, reuse (slam_toolbox)
```bash
./run_map_nav.sh slam                # build a 2D map while navigating
python3 map_tools.py save <name>     # save it any time (-> maps/<name>.*)
./run_map_nav.sh localize <name>     # later: load the map and localize in it
```
In `localize`, seed the start pose in RViz with **2D Pose Estimate** if needed.

## 3. Build / save a 3D point-cloud map only (no robot)
```bash
./run_point_lio.sh                   # walk the LiDAR through the space, Ctrl-C (wait ~3s)
cp "$(ls -t point_lio_ws/src/point_lio_ros2/PCD/*.pcd | head -1)" ~/slam_map_$(date +%F_%H%M).pcd
```

## 4. Manual driving / telemetry (no ROS)
```bash
python3 ugv_controller.py            # REPL: w/a/s/d, stop, info, raw {json}
python3 ugv_controller.py --info     # deviceInfo: r/p/y° + battery V
python3 ugv_controller.py --serve    # web control panel at http://localhost:8080/
```
Drive protocol: `GET /js?json={"T":1,"L":<l>,"R":<r>}` wheel speed m/s (−0.5..0.5);
empty reply = success. **`{"T":13}` does NOT move this unit — use T:1.**

## 5. View raw LiDAR only
```bash
./lidar_run.sh                       # driver + RViz on /unilidar/cloud
```

## 6. Quick checks
```bash
curl http://192.168.4.1/deviceInfo   # UGV01 alive? (volts + r/p/y)
ping -c3 192.168.1.62                # L2 reachable?
./test_odom.sh                       # verify Point-LIO odometry (follow prompts)
```

## Key facts
- **UGV01**: tank @ `192.168.4.1` (WiFi AP), drive `{"T":1,"L","R"}` m/s ±0.5, telemetry `/deviceInfo`.
  Footprint 0.232 × 0.197 m (set in `nav2/nav2_params.yaml`). No encoders/IMU over WiFi.
- **L2**: @ `192.168.1.62`, host `192.168.1.2/24` on `enp57s0`; `/unilidar/cloud` + `/unilidar/imu`.
  Mounted **centred left-right, ~5 cm forward of centre**, **no tilt**, scan centre **8.5 cm** up (`LIDAR_X=0.05, LIDAR_Z=0.085` in `nav2/nav2_bringup.launch.py`).
- **Point-LIO**: odom `/aft_mapped_to_init` (frames `camera_init`→`aft_mapped`), map `/cloud_registered`.
  Driver runs `use_system_timestamp:=false` (cloud+IMU must share the device clock or odometry diverges).
- **TF**: `camera_init → aft_mapped → base_link` (robot centre) — Nav2 plans/steers for `base_link`.
- Details in `UGV01_INTEGRATION.md`.
