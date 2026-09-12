#!/usr/bin/env python3
"""
nav2_bringup.launch.py -- Nav2 for the UGV01 tank on Point-LIO SLAM.

Starts:
  * pointcloud_to_laserscan : /unilidar/cloud -> /scan  (feeds the costmaps)
  * static TF               : aft_mapped -> unilidar_lidar (identity)
                              aft_mapped -> base_link (UGV01 centre, LiDAR at
                              middle-front so base_link is behind+below the LiDAR)
  * Nav2 servers            : controller (MPPI), planner (NavFn), behaviors, bt_navigator
  * lifecycle_manager       : configures + activates them (autostart)

Assumes the L2 driver, Point-LIO (frames camera_init->aft_mapped), and the UGV01
WiFi driver are already running (run_nav2.sh starts those). Nav2 controller outputs
/cmd_vel -> the UGV01 driver.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PARAMS = os.path.join(os.path.dirname(os.path.realpath(__file__)), "nav2_params.yaml")

# --- UGV01 geometry (from the Waveshare drawings, metres) --------------------
# The L2 LiDAR is mounted at the MIDDLE-FRONT of the deck, so the Point-LIO body
# frame (aft_mapped, which rides at the LiDAR) is forward of and above the true
# robot centre. base_link is the robot centre; it sits LIDAR_X behind and
# LIDAR_Z below the LiDAR. LIDAR_YAW corrects the LiDAR forward axis to the
# robot forward axis if they are not aligned (calibrate on hardware).
LIDAR_X   = 0.05     # LiDAR ~5 cm FORWARD of robot centre (centred left-right); from top-view photo
LIDAR_Z   = 0.085    # LiDAR scan centre 8.5 cm above base_link (ground plane); measured
LIDAR_YAW = 0.0      # rad; set to correct LiDAR-forward vs robot-forward mismatch

def generate_launch_description():
    lifecycle_nodes = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]

    return LaunchDescription([
        # 3D cloud -> 2D scan for the costmaps
        Node(
            package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan", output="screen",
            remappings=[("cloud_in", "/unilidar/cloud"), ("scan", "/scan")],
            parameters=[{
                # Output scan in the cloud's own frame -> no TF wait, so p2l never
                # drops messages ("queue is full"). The costmaps transform /scan.
                "target_frame": "unilidar_lidar",
                "transform_tolerance": 0.3,
                "min_height": -0.06, "max_height": 0.60,   # floor is at z=-0.085 (LiDAR 8.5cm up); cut above it so the ground isn't scanned as an obstacle
                "angle_min": -3.14159, "angle_max": 3.14159, "angle_increment": 0.0087,
                "scan_time": 0.1, "range_min": 0.20, "range_max": 8.0,
                "use_inf": True, "concurrency_level": 1,
            }],
        ),

        # Point-LIO body frame (aft_mapped) coincides with the LiDAR frame.
        # (this tf2_ros build requires the named-argument form)
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="aft_to_lidar",
             arguments=["--frame-id", "aft_mapped", "--child-frame-id", "unilidar_lidar"]),

        # Robot centre: LiDAR is at the middle-front, so base_link is LIDAR_X
        # behind and LIDAR_Z below aft_mapped. Nav2 plans/steers for base_link.
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="aft_to_base_link",
             arguments=["--x", str(-LIDAR_X), "--y", "0", "--z", str(-LIDAR_Z),
                        "--yaw", str(-LIDAR_YAW), "--pitch", "0", "--roll", "0",
                        "--frame-id", "aft_mapped", "--child-frame-id", "base_link"]),

        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[PARAMS], remappings=[("cmd_vel", "/cmd_vel")]),
        Node(package="nav2_planner", executable="planner_server", output="screen",
             parameters=[PARAMS]),
        Node(package="nav2_behaviors", executable="behavior_server", output="screen",
             parameters=[PARAMS]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", output="screen",
             parameters=[PARAMS]),

        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"autostart": True, "node_names": lifecycle_nodes}]),
    ])
