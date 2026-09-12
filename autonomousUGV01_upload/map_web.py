#!/usr/bin/env python3
"""
map_web.py -- web UI backend to save + use maps for the UGV01 + L2 stack.

Serves map_ui.html and a small JSON API that:
  * starts the stack in SLAM (new map) or LOCALIZE (on a saved map) mode
  * saves / lists / deletes maps
  * stops the stack (and the robot)

The stack itself (run_map_nav.sh) sources ROS and drives the robot; this backend
just launches/stops it and calls map_tools.py to save maps.

    python3 map_web.py           # then open http://localhost:8090/
    python3 map_web.py --port 8091 --host 0.0.0.0
"""
import argparse
import glob
import json
import os
import signal
import subprocess
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
MAPS = os.path.join(HERE, "maps")
LOGDIR = os.path.join(HERE, "logs")
os.makedirs(MAPS, exist_ok=True)
os.makedirs(LOGDIR, exist_ok=True)
ROS_SETUP = "/opt/ros/lyrical/setup.bash"

# ---- running-stack state -----------------------------------------------------
STATE = {"proc": None, "mode": None, "map": None, "started": 0.0}


def stack_running():
    p = STATE["proc"]
    return p is not None and p.poll() is None


def start_stack(mode, mapname):
    if stack_running():
        return False, "a stack is already running — stop it first"
    if mode == "localize":
        if not mapname or not os.path.exists(os.path.join(MAPS, mapname + ".posegraph")):
            return False, f"map '{mapname}' not found"
        cmd = f'exec "{HERE}/run_map_nav.sh" localize "{mapname}"'
    elif mode == "slam":
        cmd = f'exec "{HERE}/run_map_nav.sh" slam'
    else:
        return False, "mode must be 'slam' or 'localize'"
    log = open(os.path.join(LOGDIR, "stack.log"), "wb")
    # start_new_session -> own process group, so we can signal the whole tree
    proc = subprocess.Popen(["bash", "-lc", cmd], cwd=HERE, stdout=log, stderr=log,
                            start_new_session=True)
    STATE.update(proc=proc, mode=mode, map=mapname if mode == "localize" else None,
                 started=time.time())
    return True, f"started {mode}" + (f" on '{mapname}'" if mapname else "")


def stop_stack():
    if not stack_running():
        STATE.update(proc=None, mode=None, map=None)
        return False, "nothing running"
    pgid = os.getpgid(STATE["proc"].pid)
    os.killpg(pgid, signal.SIGINT)          # triggers run_map_nav.sh cleanup (stops robot)
    for _ in range(30):
        if STATE["proc"].poll() is not None:
            break
        time.sleep(0.2)
    if STATE["proc"].poll() is None:
        os.killpg(pgid, signal.SIGKILL)
    STATE.update(proc=None, mode=None, map=None)
    return True, "stopped"


def save_map(name):
    if not stack_running() or STATE["mode"] != "slam":
        return False, "save needs the SLAM (mapping) stack running"
    name = "".join(c for c in name if c.isalnum() or c in "-_") or "map"
    r = subprocess.run(["bash", "-lc",
                        f'source {ROS_SETUP}; python3 "{HERE}/map_tools.py" save "{name}"'],
                       cwd=HERE, capture_output=True, text=True, timeout=40)
    ok = os.path.exists(os.path.join(MAPS, name + ".posegraph"))
    return ok, (r.stdout + r.stderr).strip()[-400:]


def list_maps():
    out = []
    for p in sorted(glob.glob(os.path.join(MAPS, "*.posegraph"))):
        n = os.path.basename(p)[:-10]
        out.append({"name": n,
                    "kb": round(os.path.getsize(p) / 1e3),
                    "grid": os.path.exists(os.path.join(MAPS, n + ".yaml")),
                    "mtime": time.strftime("%Y-%m-%d %H:%M",
                                           time.localtime(os.path.getmtime(p)))})
    return out


def delete_map(name):
    n = 0
    for ext in (".posegraph", ".data", ".pgm", ".yaml"):
        p = os.path.join(MAPS, name + ext)
        if os.path.exists(p):
            os.remove(p); n += 1
    return n > 0, f"deleted {n} file(s)"


def status():
    return {"running": stack_running(), "mode": STATE["mode"], "map": STATE["map"],
            "uptime": round(time.time() - STATE["started"]) if stack_running() else 0}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _body(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            return {}

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path in ("/", "/index.html", "/map_ui.html"):
            try:
                with open(os.path.join(HERE, "map_ui.html"), "rb") as f:
                    body = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            except OSError:
                self._json({"error": "map_ui.html missing"}, 404)
        elif u.path == "/api/status":
            self._json(status())
        elif u.path == "/api/maps":
            self._json({"maps": list_maps(), "status": status()})
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        b = self._body()
        if u.path == "/api/start":
            ok, msg = start_stack(b.get("mode"), b.get("name"))
        elif u.path == "/api/stop":
            ok, msg = stop_stack()
        elif u.path == "/api/save":
            ok, msg = save_map(b.get("name", "map"))
        elif u.path == "/api/delete":
            ok, msg = delete_map(b.get("name", ""))
        else:
            return self._json({"error": "not found"}, 404)
        self._json({"ok": ok, "msg": msg, "status": status()})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    args = ap.parse_args()
    srv = ThreadingHTTPServer((args.host, args.port), H)
    print(f"Map UI: http://localhost:{args.port}/   (maps in {MAPS})")
    print("Ctrl-C to stop the UI (stop the robot stack from the page first).")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        if stack_running():
            stop_stack()
        print("\nbye")


if __name__ == "__main__":
    main()
