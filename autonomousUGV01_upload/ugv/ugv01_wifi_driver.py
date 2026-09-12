#!/usr/bin/env python3
"""
ugv01_wifi_driver.py -- WiFi (HTTP JSON) driver for the Waveshare UGV01 tank.

This board's WiFi firmware (UGV01_BASE_WEB) exposes (verified on hardware):
  * drive:      GET /js?json={"T":1,"L":<left m/s>,"R":<right m/s>}   (closed-loop
                wheel-speed control, range -0.5..0.5 m/s; empty reply = OK).
                NOTE: {"T":13} (ROS_CTRL) does NOT actuate this unit -- use T:1.
  * telemetry:  GET /deviceInfo -> {"V":volts,"r":roll,"p":pitch,"y":yaw,...}
It does NOT expose wheel encoders or raw IMU over WiFi, so odometry + mapping come
from the L2 LiDAR via Point-LIO. This node turns /cmd_vel (Twist) into left/right
wheel speeds using true differential-drive kinematics and the UGV01 track width.

  subscribes:  /cmd_vel (Twist)      -> {"T":1,"L","R"}  wheel speeds (m/s)
  publishes :  /voltage (Float32)    battery volts
               /ugv/rpy (Vector3)    board roll/pitch/yaw (degrees)

Kinematics (differential drive):
    v_left  = v - w * wheel_separation / 2
    v_right = v + w * wheel_separation / 2      (both clamped to +/- max_wheel m/s)

Params:  robot_ip (192.168.4.1)
         wheel_separation (m, track center-to-center; UGV01 ~0.152 geometric,
                           default 0.17 effective to compensate for track slip)
         max_wheel (m/s per track, hardware limit 0.5)
         cmd_timeout (s; stop the robot if no /cmd_vel arrives within this window)
"""
import json, math, urllib.parse, urllib.request, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

DEFAULT_IP  = "192.168.4.1"
WHEEL_SEP   = 0.17     # m, effective track separation (geometric ~0.152, +slip)
MAX_WHEEL   = 0.5      # m/s hardware limit per track
CMD_TIMEOUT = 0.4      # s, stop if no cmd_vel within this window (safety watchdog)
TELEM_HZ    = 5.0


def drive_ms(ip, left, right, timeout=0.25):
    """Send wheel speeds via {"T":1,"L","R"}. Fire-and-forget (empty reply = OK)."""
    payload = json.dumps({"T": 1, "L": left, "R": right}, separators=(",", ":"))
    try:
        urllib.request.urlopen(f"http://{ip}/js?json=" + urllib.parse.quote(payload),
                               timeout=timeout).read()
    except Exception:
        pass          # RemoteDisconnected / empty body is normal success for /js


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def rpy_deg_to_quat(roll, pitch, yaw):
    """Board roll/pitch/yaw in DEGREES -> geometry_msgs/Quaternion."""
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class UGV01Wifi(Node):
    def __init__(self):
        super().__init__("ugv01_wifi_driver")
        self.ip = self.declare_parameter("robot_ip", DEFAULT_IP).value
        self.wheel_sep = float(self.declare_parameter("wheel_separation", WHEEL_SEP).value)
        self.max_wheel = float(self.declare_parameter("max_wheel", MAX_WHEEL).value)
        self.cmd_timeout = float(self.declare_parameter("cmd_timeout", CMD_TIMEOUT).value)
        self.volt_pub = self.create_publisher(Float32, "voltage", 10)
        self.rpy_pub = self.create_publisher(Vector3, "ugv/rpy", 10)
        self.imu_pub = self.create_publisher(Imu, "ugv/imu", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / TELEM_HZ, self.telem)
        self.create_timer(0.1, self.watchdog)          # 10 Hz safety check
        self.last_cmd_t = 0.0
        self.moving = False
        self.warned = False
        self.get_logger().info(
            f"UGV01 WiFi driver -> http://{self.ip}  (cmd_vel -> T:1 L/R, "
            f"wheel_sep={self.wheel_sep} m)")

    def on_cmd(self, m):
        v = float(m.linear.x)
        w = float(m.angular.z)
        left  = clamp(v - w * self.wheel_sep / 2.0, -self.max_wheel, self.max_wheel)
        right = clamp(v + w * self.wheel_sep / 2.0, -self.max_wheel, self.max_wheel)
        drive_ms(self.ip, round(left, 3), round(right, 3))
        self.last_cmd_t = time.monotonic()
        self.moving = abs(left) > 1e-3 or abs(right) > 1e-3

    def watchdog(self):
        """Stop the robot if cmd_vel goes silent (e.g. Nav2 died mid-drive)."""
        if self.moving and (time.monotonic() - self.last_cmd_t) > self.cmd_timeout:
            drive_ms(self.ip, 0.0, 0.0)
            self.moving = False
            self.get_logger().warning("cmd_vel timeout -> stopping robot",
                                      throttle_duration_sec=5.0)

    def telem(self):
        try:
            url = f"http://{self.ip}/deviceInfo"
            d = json.loads(urllib.request.urlopen(url, timeout=0.3).read().decode())
        except Exception:
            if not self.warned:
                self.get_logger().warning(f"no telemetry from http://{self.ip} - check WiFi/IP",
                                          throttle_duration_sec=5.0)
                self.warned = True
            return
        if "V" in d:
            self.volt_pub.publish(Float32(data=float(d["V"])))
            if float(d["V"]) < 10.5:
                self.get_logger().warning(f"battery low: {d['V']} V", throttle_duration_sec=30.0)
        roll, pitch, yaw = float(d.get("r", 0.0)), float(d.get("p", 0.0)), float(d.get("y", 0.0))
        self.rpy_pub.publish(Vector3(x=roll, y=pitch, z=yaw))
        # sensor_msgs/Imu: only the board's FUSED orientation is available over WiFi
        # (no raw gyro/accel), so mark angular_velocity/linear_acceleration unknown (-1).
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "ugv_imu"
        imu.orientation = rpy_deg_to_quat(roll, pitch, yaw)
        imu.angular_velocity_covariance[0] = -1.0
        imu.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(imu)


def main():
    rclpy.init()
    n = UGV01Wifi()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        drive_ms(n.ip, 0.0, 0.0)          # stop on exit
        n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
