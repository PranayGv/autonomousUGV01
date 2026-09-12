#!/usr/bin/env python3
"""
map_tools.py -- save / list / delete 2D maps for the UGV01 + L2 stack.

Maps live in ./maps/ as:
  <name>.posegraph + <name>.data   (slam_toolbox serialized map -> relocalization)
  <name>.pgm + <name>.yaml         (occupancy grid -> viewing / map_server)

Usage (slam_toolbox mapping must be running for 'save'):
  python3 map_tools.py save  <name>
  python3 map_tools.py list
  python3 map_tools.py delete <name>
"""
import os
import sys
import glob

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, "maps")
os.makedirs(MAPS, exist_ok=True)


def cmd_list():
    names = sorted(os.path.basename(p)[:-10]
                   for p in glob.glob(os.path.join(MAPS, "*.posegraph")))
    if not names:
        print("(no maps saved in %s)" % MAPS)
    for n in names:
        pgm = "grid" if os.path.exists(os.path.join(MAPS, n + ".yaml")) else "-"
        mb = os.path.getsize(os.path.join(MAPS, n + ".posegraph")) / 1e3
        print(f"  {n}\t({mb:.0f} kB, {pgm})")
    return names


def cmd_delete(name):
    n = 0
    for ext in (".posegraph", ".data", ".pgm", ".yaml"):
        p = os.path.join(MAPS, name + ext)
        if os.path.exists(p):
            os.remove(p); n += 1
    print(f"deleted {n} file(s) for '{name}'" if n else f"no files for '{name}'")


def cmd_save(name):
    import rclpy
    from rclpy.node import Node
    from slam_toolbox.srv import SerializePoseGraph, SaveMap
    from std_msgs.msg import String

    base = os.path.join(MAPS, name)
    rclpy.init()
    node = Node("map_saver_cli")

    def call(cli, req, label):
        """Returns (ok, result_code|None). ok means service replied with result==0."""
        if not cli.wait_for_service(timeout_sec=5.0):
            print(f"ERROR: {label}: service not available — is the SLAM stack running?")
            return False, None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=25.0)
        if fut.result() is None:
            print(f"ERROR: {label}: call timed out"); return False, None
        code = fut.result().result
        return code == 0, code

    ser = SerializePoseGraph.Request(); ser.filename = base
    ok1, c1 = call(node.create_client(SerializePoseGraph, "/slam_toolbox/serialize_map"),
                   ser, "serialize (relocalization map)")

    sm = SaveMap.Request(); sm.name = String(data=base)
    ok2, c2 = call(node.create_client(SaveMap, "/slam_toolbox/save_map"),
                   sm, "save_map (occupancy grid)")

    node.destroy_node(); rclpy.shutdown()

    have_file = os.path.exists(base + ".posegraph")
    if ok1 and have_file:
        print(f"OK: saved map '{name}' -> {base}.posegraph (+ .data)"
              + (f" + {base}.pgm/.yaml" if ok2 else ""))
        return 0
    # explain the common failure clearly
    if c2 == 1 or (not ok1 and c1 is None and not have_file):
        print("FAILED: slam_toolbox has NO MAP yet. Start the SLAM stack, let it come "
              "up (~30 s), then DRIVE THE ROBOT AROUND so it builds a map (watch the "
              "map appear in RViz) before saving.")
    else:
        print(f"FAILED to save '{name}' (serialize result={c1}, save_map result={c2}).")
    return 1


def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    op = sys.argv[1]
    if op == "list":
        cmd_list(); return 0
    if op == "save" and len(sys.argv) == 3:
        return cmd_save(sys.argv[2])
    if op == "delete" and len(sys.argv) == 3:
        cmd_delete(sys.argv[2]); return 0
    print(__doc__); return 1


if __name__ == "__main__":
    sys.exit(main())
