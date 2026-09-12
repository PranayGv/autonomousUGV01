#!/usr/bin/env python3
"""
UGV01 JSON command controller  (verified against real UGV01_BASE_WEB firmware).

How the robot's own web UI actually works (reverse-engineered from
http://192.168.4.1/ ):

  * Send a JSON command:   GET /js?json=<json>
        - The firmware processes the command but returns an EMPTY body and
          closes the socket (Python raises RemoteDisconnected). That is NORMAL
          and means success -- it is NOT an error.
  * Read command feedback: GET /jsfb
        - Returns the JSON/text feedback produced by the LAST /js command
          (e.g. T:902 -> wheel rates, T:904 -> speed rate). Empty for commands
          that produce no feedback.
  * Live telemetry:        GET /deviceInfo
        - Returns e.g. {"V":11.4,"r":6.5,"p":-1.3,"y":-140.5,
                        "mX":..,"mY":..,"mZ":..,"IP":..,"MAC":..,"RSSI":..,"SPEED":..}
          r/p/y = roll/pitch/yaw angles (deg), V = battery volts,
          m* = magnetometer, SPEED = wheel speed feedback.
  * Simple movement:       GET /cmd?inputA=1&inputB=<L>&inputC=<R>
        - Used by the on-page D-pad (inputA=1 -> speed mode).

Reference: https://www.waveshare.com/wiki/UGV01

Usage
-----
Library:
    from ugv_controller import UGV01
    ugv = UGV01("192.168.4.1")
    ugv.ros_ctrl(x=0.15, z=0.0)     # drive
    print(ugv.device_info())        # angles / voltage / speed
    print(ugv.command({"T":902}))   # send + read jsfb feedback
    ugv.stop()

REPL:              python3 ugv_controller.py
One-shot:          python3 ugv_controller.py --cmd '{"T":126}'
Live telemetry:    python3 ugv_controller.py --info
Web UI + proxy:    python3 ugv_controller.py --serve   ->  http://localhost:8080/
"""

import argparse
import json
import urllib.parse
import urllib.request
import urllib.error
import http.client


class UGV01:
    def __init__(self, ip="192.168.4.1", timeout=3.0):
        self.ip = ip
        self.timeout = timeout

    # ---- low-level HTTP GET ---------------------------------------------
    def _get(self, path):
        """Raw GET. Returns response text ('' if the robot closed w/o a body)."""
        url = "http://{}/{}".format(self.ip, path)
        try:
            with urllib.request.urlopen(url, timeout=self.timeout) as resp:
                return resp.read().decode("utf-8", "replace").strip()
        except (http.client.RemoteDisconnected, urllib.error.URLError) as exc:
            # /js closes without a proper HTTP reply after executing the command.
            reason = getattr(exc, "reason", exc)
            if isinstance(exc, http.client.RemoteDisconnected) or isinstance(
                reason, http.client.RemoteDisconnected
            ):
                return ""  # command executed; no body -> success
            raise

    # ---- command transport ----------------------------------------------
    def send(self, command: dict):
        """Send a JSON command (GET /js?json=...). Returns {'ok':True} on success."""
        raw = json.dumps(command, separators=(",", ":"))
        try:
            self._get("js?json=" + urllib.parse.quote(raw, safe="{}\":,.-"))
            return {"ok": True, "sent": command}
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc), "sent": command}

    def feedback(self):
        """GET /jsfb -> feedback of the most recent command (parsed if JSON)."""
        try:
            body = self._get("jsfb")
        except Exception as exc:  # noqa: BLE001
            return {"error": str(exc)}
        if not body:
            return None
        try:
            return json.loads(body)
        except json.JSONDecodeError:
            return {"raw": body}

    def command(self, command: dict):
        """Send a command AND return its /jsfb feedback (for query commands)."""
        self.send(command)
        return self.feedback()

    def device_info(self):
        """GET /deviceInfo -> live telemetry dict (angles, voltage, speed, mag)."""
        try:
            body = self._get("deviceInfo")
            return json.loads(body) if body else None
        except Exception as exc:  # noqa: BLE001
            return {"error": str(exc)}

    def cmd(self, input_a=1, input_b=0, input_c=0):
        """GET /cmd?inputA=&inputB=&inputC=  (simple movement, as the D-pad uses)."""
        try:
            self._get("cmd?inputA={}&inputB={}&inputC={}".format(input_a, input_b, input_c))
            return {"ok": True}
        except Exception as exc:  # noqa: BLE001
            return {"ok": False, "error": str(exc)}

    # ---- movement -------------------------------------------------------
    def speed(self, left, right):
        """CMD_SPEED_CTRL (T:1). Wheel speeds -0.5..+0.5 m/s."""
        return self.send({"T": 1, "L": left, "R": right})

    def ros_ctrl(self, x, z):
        """CMD_ROS_CTRL (T:13). NOTE: verified NOT to actuate this UGV01 unit;
        kept for completeness. Use speed()/forward()/turn_* (T:1) to actually move."""
        return self.send({"T": 13, "X": x, "Z": z})

    def stop(self):
        return self.send({"T": 1, "L": 0, "R": 0})

    # Directional helpers use T:1 differential wheel speed (m/s) -- the command
    # this firmware actually responds to.
    def forward(self, s=0.2):
        return self.speed(s, s)

    def backward(self, s=0.2):
        return self.speed(-s, -s)

    def turn_left(self, s=0.2):
        return self.speed(-s, s)

    def turn_right(self, s=0.2):
        return self.speed(s, -s)

    # ---- telemetry helpers ----------------------------------------------
    def get_imu(self):
        """Angles/attitude. Robust: sends T:126 (jsfb) and falls back to deviceInfo."""
        fb = self.command({"T": 126})
        info = self.device_info() or {}
        out = {}
        if isinstance(info, dict):
            out.update({k: info[k] for k in ("r", "p", "y", "mX", "mY", "mZ", "V", "SPEED")
                        if k in info})
        if isinstance(fb, dict):
            out["jsfb"] = fb
        return out or fb

    def get_base_feedback(self):
        """Wheel rate / speed feedback: T:902 -> jsfb, plus SPEED from deviceInfo."""
        fb = self.command({"T": 902})
        info = self.device_info() or {}
        if isinstance(fb, dict) and isinstance(info, dict) and "SPEED" in info:
            fb["SPEED"] = info["SPEED"]
        return fb

    # ---- peripherals ----------------------------------------------------
    def set_pid(self, p=170, i=90):
        return self.send({"T": 2, "P": p, "I": i})

    def oled(self, line, text):
        return self.send({"T": 3, "lineNum": line, "Text": text})

    def oled_reset(self):
        return self.send({"T": -3})

    def led(self, value):
        return self.send({"T": 114, "led": value})


# =========================================================================
# Interactive REPL
# =========================================================================
HELP = """\
  w/f [spd]        forward         s/b [spd]   backward
  a [rate]         turn left       d [rate]    turn right
  x / space        stop
  spd <l> <r>      wheel speeds (m/s, -0.5..0.5)
  ros <x> <z>      linear x (m/s), angular z (rad/s)
  info             deviceInfo (angles/voltage/speed)
  imu              angles + T:126 feedback
  fb               base/wheel feedback (T:902)
  led <0-255>      LED         oled <n> <text> / oledreset
  raw <json>       raw command, e.g. raw {"T":126}   (then shows /jsfb)
  help             this help                          q  quit
"""


def _p(x):
    print(json.dumps(x, indent=2) if isinstance(x, (dict, list)) else x)


def repl(ugv: UGV01):
    print("UGV01 -> http://{} | 'help' for commands, 'q' to quit\n".format(ugv.ip))
    while True:
        try:
            line = input("ugv> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            ugv.stop()
            break
        if not line:
            continue
        parts = line.split()
        c, a = parts[0].lower(), parts[1:]
        try:
            if c in ("q", "quit", "exit"):
                ugv.stop()
                break
            elif c in ("help", "?"):
                print(HELP)
            elif c in ("w", "f"):
                _p(ugv.forward(float(a[0]) if a else 0.2))
            elif c in ("s", "b"):
                _p(ugv.backward(float(a[0]) if a else 0.2))
            elif c == "a":
                _p(ugv.turn_left(float(a[0]) if a else 0.3))
            elif c == "d":
                _p(ugv.turn_right(float(a[0]) if a else 0.3))
            elif c in ("x", "stop"):
                _p(ugv.stop())
            elif c == "spd":
                _p(ugv.speed(float(a[0]), float(a[1])))
            elif c == "ros":
                _p(ugv.ros_ctrl(float(a[0]), float(a[1])))
            elif c == "info":
                _p(ugv.device_info())
            elif c == "imu":
                _p(ugv.get_imu())
            elif c in ("fb", "feedback"):
                _p(ugv.get_base_feedback())
            elif c == "led":
                _p(ugv.led(int(a[0])))
            elif c == "oled":
                _p(ugv.oled(int(a[0]), " ".join(a[1:])))
            elif c == "oledreset":
                _p(ugv.oled_reset())
            elif c == "raw":
                _p(ugv.command(json.loads(line[3:].strip())))
            else:
                print("unknown; type 'help'")
        except (IndexError, ValueError) as exc:
            print("bad args:", exc)


# =========================================================================
# Static page + CORS-free proxy (so the HTML can READ telemetry)
# =========================================================================
def serve(ugv: UGV01, host="0.0.0.0", port=8080, html_path="ugv_control.html"):
    import http.server
    import os

    PROXY = {"js", "jsfb", "deviceInfo", "cmd"}

    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="text/plain"):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            u = urllib.parse.urlparse(self.path)
            seg = u.path.strip("/")
            if seg in ("", "index.html", os.path.basename(html_path)):
                try:
                    with open(html_path, "rb") as fh:
                        self._send(200, fh.read(), "text/html; charset=utf-8")
                except OSError:
                    self._send(404, "ugv_control.html not found next to the script")
            elif seg in PROXY:
                # Forward path+query straight to the robot, relay whatever comes back.
                fwd = seg + (("?" + u.query) if u.query else "")
                try:
                    body = ugv._get(fwd)
                except Exception as exc:  # noqa: BLE001
                    self._send(200, json.dumps({"error": str(exc)}), "application/json")
                    return
                self._send(200, body if body else "", "application/json")
            else:
                self._send(404, "not found")

    httpd = http.server.HTTPServer((host, port), Handler)
    print("UGV01 web UI:  http://localhost:{}/   (robot {})".format(port, ugv.ip))
    print("Ctrl-C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        ugv.stop()
        print("\nstopped")


def main():
    ap = argparse.ArgumentParser(description="UGV01 JSON command controller")
    ap.add_argument("--ip", default="192.168.4.1")
    ap.add_argument("--timeout", type=float, default=3.0)
    ap.add_argument("--cmd", help="send one raw JSON command, show /jsfb, exit")
    ap.add_argument("--info", action="store_true", help="print deviceInfo and exit")
    ap.add_argument("--serve", action="store_true", help="serve HTML UI + proxy")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    ugv = UGV01(args.ip, timeout=args.timeout)
    if args.cmd:
        _p(ugv.command(json.loads(args.cmd)))
    elif args.info:
        _p(ugv.device_info())
    elif args.serve:
        import os
        html = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ugv_control.html")
        serve(ugv, port=args.port, html_path=html)
    else:
        repl(ugv)


if __name__ == "__main__":
    main()
