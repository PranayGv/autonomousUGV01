#!/usr/bin/env python3
"""
capture_360.py -- accumulate the Unitree L2 point cloud over a few seconds into
one dense cloud, save it to .pcd, and report the actual horizontal (azimuth) and
vertical (elevation) coverage so you can confirm it is a full 360 deg ring.

The L2 is a non-repetitive dome scanner (360 deg horizontal x ~90 deg vertical),
so a single frame is sparse; accumulating fills in the full field of view.

Run it with the driver already publishing /unilidar/cloud (see capture_360.sh,
which launches the driver at FULL FOV -- rear_blind_deg:=0.0 -- first).

    source /opt/ros/lyrical/setup.bash
    python3 capture_360.py --seconds 6 --topic /unilidar/cloud --out cloud360.pcd
"""
import argparse
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


_NP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
       5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def parse_xyz(msg: PointCloud2):
    """Manually decode x,y,z (and intensity if present) from a PointCloud2."""
    off = {f.name: (f.offset, f.datatype) for f in msg.fields}
    need = ("x", "y", "z")
    if not all(k in off for k in need):
        return None, None
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    n = msg.width * msg.height
    buf = buf.reshape(n, msg.point_step)

    def col(name):
        o, dt = off[name]
        npdt = _NP[dt]
        raw = buf[:, o:o + np.dtype(npdt).itemsize].copy()
        return raw.view(npdt).reshape(-1)

    xyz = np.stack([col("x"), col("y"), col("z")], axis=1).astype(np.float32)
    inten = col("intensity").astype(np.float32) if "intensity" in off else None
    return xyz, inten


class Capture(Node):
    def __init__(self, topic, seconds):
        super().__init__("capture_360")
        self.seconds = seconds
        self.clouds = []
        self.inten = []
        self.frames = 0
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, qos)
        self.t0 = None
        self.get_logger().info("waiting for '%s' ..." % topic)

    def cb(self, msg):
        xyz, inten = parse_xyz(msg)
        if xyz is None or xyz.size == 0:
            return
        m = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 0)
        xyz = xyz[m]
        if self.t0 is None:
            self.t0 = time.time()
            self.get_logger().info("receiving; accumulating %.1f s ..." % self.seconds)
        self.clouds.append(xyz)
        if inten is not None:
            self.inten.append(inten[m])
        self.frames += 1


def save_pcd(path, xyz, inten):
    n = xyz.shape[0]
    has_i = inten is not None and len(inten) == n
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data\nVERSION 0.7\n")
        if has_i:
            f.write("FIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n")
        else:
            f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        f.write("WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n"
                % (n, n))
        if has_i:
            for p, i in zip(xyz, inten):
                f.write("%.4f %.4f %.4f %.2f\n" % (p[0], p[1], p[2], i))
        else:
            for p in xyz:
                f.write("%.4f %.4f %.4f\n" % (p[0], p[1], p[2]))


def coverage_report(xyz):
    az = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))          # -180..180
    r_xy = np.hypot(xyz[:, 0], xyz[:, 1])
    el = np.degrees(np.arctan2(xyz[:, 2], r_xy))               # -90..90
    # occupancy over 5-deg azimuth bins
    bins = np.zeros(72, dtype=bool)
    bins[((az + 180) // 5).astype(int).clip(0, 71)] = True
    covered = bins.sum()
    print("\n=== COVERAGE ===")
    print("points total          : %d" % xyz.shape[0])
    print("azimuth span          : %.1f deg  (min %.1f, max %.1f)"
          % (az.max() - az.min(), az.min(), az.max()))
    print("azimuth bins occupied : %d / 72  (%.0f%% of 360 deg)"
          % (covered, 100.0 * covered / 72))
    print("elevation span        : %.1f deg  (min %.1f, max %.1f)"
          % (el.max() - el.min(), el.min(), el.max()))
    if covered >= 69:
        print(">> FULL 360 deg horizontal coverage confirmed.")
    elif covered >= 40:
        print(">> Partial ring (~%.0f deg). If a wedge is missing, a SLAM launch"
              " set rear_blind_deg>0 -- use full FOV (rear_blind_deg:=0)." % (5 * covered))
    else:
        print(">> Narrow coverage -- check that the driver is the full-FOV one.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/unilidar/cloud")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--out", default="cloud360.pcd")
    args = ap.parse_args()

    rclpy.init()
    node = Capture(args.topic, args.seconds)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.t0 and (time.time() - node.t0) >= args.seconds:
                break
    finally:
        pass

    if not node.clouds:
        print("ERROR: no points received on %s. Is the driver running?" % args.topic)
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    xyz = np.concatenate(node.clouds, axis=0)
    inten = np.concatenate(node.inten, axis=0) if node.inten else None
    print("accumulated %d frames -> %d points" % (node.frames, xyz.shape[0]))
    save_pcd(args.out, xyz, inten)
    print("saved: %s" % args.out)
    coverage_report(xyz)
    node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
