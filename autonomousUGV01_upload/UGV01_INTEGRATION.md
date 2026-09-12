# UGV01 tank — SLAM & navigation integration

Fusing the Waveshare **UGV01** tracked base (WiFi JSON API) with the **Unitree L2**
4D LiDAR for autonomous goal navigation + obstacle avoidance on ROS 2 Lyrical.

## How it actually works (verified on hardware)
The UGV01 board exposes **no wheel encoders or IMU over WiFi**, so all odometry and
mapping come from the **L2 LiDAR via Point-LIO** (LiDAR-inertial odometry). The tank
is a pure actuator: Nav2 produces `/cmd_vel`, the WiFi driver converts it to left/right
track speeds and sends them over HTTP.

```
                 goal (RViz "2D Goal Pose")
                          │
L2 LiDAR ─► Point-LIO ─► odom camera_init→aft_mapped  +  3D map
   │  /unilidar/cloud     │
   │                      ├─► pointcloud_to_laserscan ─► /scan ─► Nav2 costmaps
   │                      │
   └──────────────────────┴─► Nav2 (MPPI controller + NavFn planner) ─► /cmd_vel
                                                                          │
                          ugv/ugv01_wifi_driver.py  ── HTTP {"T":1,L,R} ─► UGV01 @ 192.168.4.1
```

## UGV01 WiFi drive protocol (verified)
- **Drive:** `GET /js?json={"T":1,"L":<left m/s>,"R":<right m/s>}` — closed-loop wheel
  speed, range **−0.5..+0.5 m/s**. Empty reply/closed socket = success.
  ⚠️ `{"T":13}` (ROS_CTRL) does **NOT** move this unit — always use **T:1**.
- **Telemetry:** `GET /deviceInfo` → `{"V":volts,"r":roll,"p":pitch,"y":yaw,...}`
  (published as `/voltage` and `/ugv/rpy`). No encoders/odometry here.
- The driver runs a **watchdog**: if `/cmd_vel` stops for >0.4 s it sends a stop, so a
  dead Nav2 can't leave the tank running.

## UGV01 geometry (from the Waveshare drawings)
| Quantity | Value | Where it's set |
|----------|-------|----------------|
| Body footprint (L×W) | 0.232 m × 0.197 m (half-extents 0.116 × 0.098) | `nav2/nav2_params.yaml` `footprint` |
| Height | 0.091 m | — |
| Track width (each) | 0.0445 m | — |
| Track separation (centre-to-centre) | 0.152 m geometric → **0.17 m effective** | driver `wheel_separation` |
| **LiDAR mount** (middle-front) | **+0.06 m forward, +0.14 m up** from robot centre | `nav2/nav2_bringup.launch.py` `LIDAR_X/Z` |

The LiDAR sits at the **middle-front**, so Point-LIO's body frame `aft_mapped` (which
rides at the LiDAR) is forward of and above the true centre. A static TF
`aft_mapped → base_link` places `base_link` at the robot centre; Nav2 plans and steers
for `base_link` with the full rectangular footprint. TF tree:
```
camera_init → aft_mapped → { unilidar_lidar (identity) , base_link (−0.06,0,−0.14) }
```

## Run it
```bash
cd ~/Documents/Projects/sentrix/slamnav
sudo ./lidar_setup_ethernet.sh        # L2 host IP 192.168.1.2 (once per boot)
# make sure this machine is ALSO on the UGV01 WiFi AP (SSID UGV / 12345678)
./run_nav2.sh
```
Keep the robot **still ~5 s** at start for Point-LIO IMU init. Then in RViz use
**2D Goal Pose** — the tank plans a path and drives there, avoiding obstacles.
`Ctrl-C` stops everything (and the robot).

## Persistent maps (save + relocalize)
Nav2 now plans in a **`map`** frame provided by **slam_toolbox** (layered on Point-LIO
odometry). Three ways to run:

| Command | What it does |
|---------|--------------|
| `./run_nav2.sh` | live rolling nav, **no saved map** (`map`≡`camera_init` identity) |
| `./run_map_nav.sh slam` | build a **new** 2D map while navigating — then save it |
| `./run_map_nav.sh localize <name>` | load `maps/<name>` and **localize** in it |

Save the current map (while `run_map_nav.sh slam` is running):
```bash
python3 map_tools.py save lab_floor       # -> maps/lab_floor.posegraph/.data/.pgm/.yaml
python3 map_tools.py list                  # list saved maps
```
Reuse it later: `./run_map_nav.sh localize lab_floor`. If the robot doesn't start where
mapping began, seed its pose with RViz **“2D Pose Estimate.”**

**Web UI** — save/list/delete maps and start/stop SLAM or localization from a browser:
```bash
python3 map_web.py                         # then open http://localhost:8090/
```
Frames with maps: `map → camera_init (Point-LIO odom) → aft_mapped → base_link`.
slam_toolbox config: `nav2/slam_toolbox_mapping.yaml`, `nav2/slam_toolbox_localization.yaml`.

## Calibration knobs (tune on hardware)
- **Turning too little / too much** → adjust `wheel_separation` in `ugv/ugv01_wifi_driver.py`
  (bigger = more aggressive turns; tracked skid-steer usually needs > geometric 0.152).
- **Robot drives the wrong heading vs the map** → the LiDAR's forward axis isn't the
  robot's forward axis: set `LIDAR_YAW` (rad) in `nav2/nav2_bringup.launch.py`.
- **LiDAR fore/aft/up position** → `LIDAR_X` / `LIDAR_Z` in the same file.
- **Won't fit through gaps** → lower `inflation_radius` in `nav2/nav2_params.yaml`.

## Files
| File | Role |
|------|------|
| `run_nav2.sh` | full stack: L2 driver + Point-LIO + WiFi driver + goal relay + Nav2 |
| `ugv/ugv01_wifi_driver.py` | `/cmd_vel` → `{"T":1,L,R}` (diff-drive), telemetry, watchdog |
| `nav2/nav2_bringup.launch.py` | pointcloud→scan, base_link/LiDAR TF, Nav2 servers |
| `nav2/nav2_params.yaml` | costmaps, footprint, MPPI/NavFn tuning |
| `nav2/goal_relay.py` | RViz `/goal_pose` → Nav2 `NavigateToPose` action |
