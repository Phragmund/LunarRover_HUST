#!/usr/bin/env python3
"""Phone web-joystick remote for the lunar rover.

Serves a touch joystick page and publishes lunar_rover_msgs/ManualControl
(LX/LY/RX 杆量) to /manual_control, the same topic the keyboard controller uses.
The motion_solver node subscribes to it and drives the H7.

Control model:
  * Joystick horizontal -> LX (right positive, Ackermann steering).
  * Joystick vertical   -> LY (up positive, throttle).
  * Spin hold buttons   -> RX (+right spin / -left spin).
  * Estop button        -> latched zero until the arm button is pressed.
"""

import asyncio
import json
import math
import secrets
import socket
import sys
import threading
import time
from contextlib import asynccontextmanager

import rclpy
from lunar_rover_msgs.msg import ManualControl
from rclpy.node import Node

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
except ImportError as exc:  # pragma: no cover
    sys.stderr.write(
        """web_joystick_server 缺少 Web 依赖 (fastapi / uvicorn)。请先安装：
  pip3 install --user 'uvicorn[standard]' fastapi
"""
    )
    raise SystemExit(1) from exc


HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no, viewport-fit=cover">
<title>月球车遥控</title>
<style>
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none; -webkit-tap-highlight-color: transparent; }
  html, body { margin: 0; height: 100%; background: #0f172a; color: #e2e8f0; font-family: system-ui, -apple-system, "Segoe UI", Roboto, sans-serif; }
  body { display: flex; flex-direction: column; align-items: center; padding: max(12px, env(safe-area-inset-top)) 16px calc(16px + env(safe-area-inset-bottom)); }
  header { width: 100%; max-width: 480px; display: flex; align-items: center; justify-content: space-between; margin-bottom: 8px; }
  h1 { font-size: 18px; margin: 0; }
  #status { font-size: 13px; padding: 4px 10px; border-radius: 999px; background: #334155; color: #94a3b8; }
  #status.on { background: #052e16; color: #22c55e; }
  #readout { width: 100%; max-width: 480px; text-align: center; color: #94a3b8; font-size: 13px; margin: 4px 0 12px; font-variant-numeric: tabular-nums; }
  #readout b { color: #e2e8f0; }
  #padwrap { position: relative; width: min(72vw, 300px); aspect-ratio: 1; margin: 8px 0 18px; }
  #pad { position: absolute; inset: 0; border-radius: 50%; background: radial-gradient(circle at 50% 40%, #24334a, #16202f); box-shadow: 0 10px 30px rgba(0,0,0,.5), inset 0 2px 6px rgba(255,255,255,.06); touch-action: none; }
  #knob { position: absolute; left: 50%; top: 50%; width: 44%; aspect-ratio: 1; margin: -22% 0 0 -22%; border-radius: 50%; background: linear-gradient(145deg, #3b4b63, #27354a); box-shadow: 0 6px 16px rgba(0,0,0,.55); }
  .hint { position: absolute; color: #64748b; font-size: 11px; }
  .hint.top { left: 50%; top: 6%; transform: translateX(-50%); }
  .hint.bottom { left: 50%; bottom: 6%; transform: translateX(-50%); }
  .hint.left { left: 8%; top: 50%; transform: translateY(-50%); }
  .hint.right { right: 8%; top: 50%; transform: translateY(-50%); }
  .row { width: 100%; max-width: 480px; display: flex; gap: 12px; margin: 6px 0; }
  button { flex: 1; height: 64px; border: none; border-radius: 14px; font-size: 16px; font-weight: 600; color: #fff; touch-action: none; cursor: pointer; }
  .spin { background: #1d4ed8; }
  .spin.active { background: #3b82f6; box-shadow: inset 0 0 0 3px #93c5fd; }
  #estop { background: #dc2626; }
  #estop.latched { box-shadow: inset 0 0 0 3px #fca5a5; }
  #arm { background: #15803d; }
  #autoBtn { background: #1e293b; }
  #autoBtn.on { background: #0e7490; box-shadow: inset 0 0 0 3px #67e8f9; }
  .note { width: 100%; max-width: 480px; color: #64748b; font-size: 12px; line-height: 1.7; margin-top: 12px; }
</style>
</head>
<body>
  <header><h1>月球车遥控</h1><span id="status">连接中…</span></header>
  <div id="readout">LX <b id="lx">0</b> · LY <b id="ly">0</b> · RX <b id="rx">0</b> · <b id="mode">stop</b></div>
  <div id="padwrap">
    <div id="pad">
      <div id="knob"></div>
      <span class="hint top">前</span><span class="hint bottom">后</span>
      <span class="hint left">左</span><span class="hint right">右</span>
    </div>
  </div>
  <div class="row">
    <button class="spin" id="spinL">⟲ 原地左转</button>
    <button class="spin" id="spinR">⟳ 原地右转</button>
  </div>
  <div class="row">
    <button id="estop">⛔ 急停</button>
    <button id="arm">✅ 解除急停</button>
  </div>
  <div class="row">
    <button id="autoBtn">💻 电脑接管</button>
  </div>
  <div class="row">
    <button id="relaxBtn" style="background:#b45309;">🔌 断开转矩 (R)</button>
    <button id="rearmBtn" style="background:#0e7490;">🔄 重启设零 (T)</button>
  </div>
  <div class="row">
    <button id="calibBtn" style="background:#7c3aed;">🛠 标定四轮 (C)</button>
  </div>
  <div class="note">摇杆上下=前进/后退；左右=阿克曼转向（杆量越大转弯半径越小）。原地转向请按住左右按键。松手即停，切后台或断连自动停。</div>
  <script>
    var statusEl = document.getElementById('status');
    var lxEl = document.getElementById('lx');
    var lyEl = document.getElementById('ly');
    var rxEl = document.getElementById('rx');
    var modeEl = document.getElementById('mode');
    var pad = document.getElementById('pad');
    var knob = document.getElementById('knob');
    var spinL = document.getElementById('spinL');
    var spinR = document.getElementById('spinR');
    var estopBtn = document.getElementById('estop');
    var armBtn = document.getElementById('arm');
    var autoBtn = document.getElementById('autoBtn');

    var ws = null;
    function connect() {
      ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws' + location.search);
      ws.onopen = function () { statusEl.textContent = '已连接'; statusEl.classList.add('on'); };
      ws.onclose = function () { statusEl.textContent = '已断开'; statusEl.classList.remove('on'); setTimeout(connect, 1500); };
      ws.onerror = function () { try { ws.close(); } catch (e) {} };
      ws.onmessage = function (ev) {
        try {
          var m = JSON.parse(ev.data);
          if (m.t === 'status') {
            lxEl.textContent = m.lx;
            lyEl.textContent = m.ly;
            rxEl.textContent = m.rx;
            modeEl.textContent = m.estop ? 'ESTOP' : m.mode;
            estopBtn.classList.toggle('latched', !!m.estop);
            autoBtn.classList.toggle('on', !!m.auto);
          }
        } catch (e) {}
      };
    }
    function send(o) { if (ws && ws.readyState === 1) { ws.send(JSON.stringify(o)); } }

    var dragging = false;
    function padCenter() {
      var r = pad.getBoundingClientRect();
      return { x: r.left + r.width / 2, y: r.top + r.height / 2, R: r.width / 2 };
    }
    function onMove(cx, cy) {
      var c = padCenter();
      var dx = cx - c.x, dy = cy - c.y;
      // 方形映射：x/y 各自独立夹到 [-1,1]，四角可同时打到 ±255
      var x = Math.max(-1, Math.min(1, dx / c.R));
      var y = Math.max(-1, Math.min(1, -(dy / c.R)));
      var rr = Math.hypot(x, y);
      var vk = rr > 1 ? 1 / rr : 1;
      knob.style.transform = 'translate(' + (x * vk * c.R) + 'px, ' + (y * vk * c.R) + 'px)';
      send({ t: 'joy', x: x, y: y });
    }
    function joyEnd() {
      dragging = false;
      knob.style.transform = 'translate(0px, 0px)';
      send({ t: 'joy', x: 0, y: 0 });
    }
    pad.addEventListener('touchstart', function (e) { e.preventDefault(); dragging = true; onMove(e.touches[0].clientX, e.touches[0].clientY); }, { passive: false });
    pad.addEventListener('touchmove', function (e) { e.preventDefault(); if (dragging) { onMove(e.touches[0].clientX, e.touches[0].clientY); } }, { passive: false });
    pad.addEventListener('touchend', joyEnd);
    pad.addEventListener('touchcancel', joyEnd);
    pad.addEventListener('mousedown', function (e) { dragging = true; onMove(e.clientX, e.clientY); });
    window.addEventListener('mousemove', function (e) { if (dragging) { onMove(e.clientX, e.clientY); } });
    window.addEventListener('mouseup', joyEnd);

    function bindHold(btn, dir) {
      var down = function (e) { e.preventDefault(); btn.classList.add('active'); send({ t: 'spin', dir: dir, on: true }); };
      var up = function () { btn.classList.remove('active'); send({ t: 'spin', dir: dir, on: false }); };
      btn.addEventListener('pointerdown', down);
      btn.addEventListener('pointerup', up);
      btn.addEventListener('pointercancel', up);
      btn.addEventListener('pointerleave', up);
      btn.addEventListener('touchstart', function (e) { e.preventDefault(); }, { passive: false });
    }
    bindHold(spinL, 'left');
    bindHold(spinR, 'right');

    setInterval(function () {
      if (dragging || spinL.classList.contains('active') || spinR.classList.contains('active')) {
        send({ t: 'heartbeat' });
      }
    }, 100);

    estopBtn.addEventListener('click', function () { send({ t: 'estop' }); });
    armBtn.addEventListener('click', function () { send({ t: 'arm' }); });
    autoBtn.addEventListener('click', function () { send({ t: 'toggle_auto' }); });
    document.getElementById('relaxBtn').addEventListener('click', function () { send({ t: 'relax' }); });
    document.getElementById('rearmBtn').addEventListener('click', function () { send({ t: 'rearm' }); });
    document.getElementById('calibBtn').addEventListener('click', function () { send({ t: 'calibrate' }); });

    document.addEventListener('visibilitychange', function () {
      if (document.hidden) {
        joyEnd();
        send({ t: 'spin', dir: 'left', on: false });
        send({ t: 'spin', dir: 'right', on: false });
      }
    });

    connect();
  </script>
</body>
</html>
"""


def _clamp(value, low, high):
    return max(low, min(high, value))


class WebJoystickServer:
    def __init__(self):
        node = Node("web_joystick_server")
        self.node = node

        self.command_topic = node.declare_parameter("command_topic", "/manual_control").value
        self.access_token = str(node.declare_parameter("access_token", "").value)
        if not self.access_token:
            node.get_logger().warn(
                "Web joystick access_token is empty; every host on this network can connect"
            )
        self.port = int(node.declare_parameter("port", 8000).value)
        self.publish_rate = float(node.declare_parameter("publish_rate", 50.0).value)
        self.deadzone = float(node.declare_parameter("deadzone", 0.05).value)
        self.deadman_timeout = float(node.declare_parameter("deadman_timeout", 0.5).value)
        self.idle_grace_period = float(node.declare_parameter("idle_grace_period", 2.0).value)

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be positive")

        self.publisher = node.create_publisher(ManualControl, self.command_topic, 1)

        self.state = {
            "nx": 0.0,
            "ny": 0.0,
            "spin_left": False,
            "spin_right": False,
            "estop": False,
            "relax_pending": False,
            "rearm_pending": False,
            "calibrate_pending": False,
            "auto_takeover": False,
            "last_input": time.monotonic(),
            "last_disconnect": -1.0e9,
            "clients": 0,
            "last_published": {"lx": 0, "ly": 0, "rx": 0, "mode": "stop", "estop": False},
        }
        self.lock = threading.Lock()
        self.connections = []

        self.app = FastAPI(lifespan=self._lifespan)
        self._register_routes()

    # ---------------- control ----------------

    def compute_sticks(self, nx, ny, spin_left, spin_right, estop, stale):
        if estop:
            return 0, 0, 0, "estop"
        if stale:
            return 0, 0, 0, "deadman"
        if spin_left:
            return 0, 0, -255, "spin left"
        if spin_right:
            return 0, 0, 255, "spin right"

        sx = nx if abs(nx) >= self.deadzone else 0.0
        sy = ny if abs(ny) >= self.deadzone else 0.0
        sx = _clamp(sx, -1.0, 1.0)
        sy = _clamp(sy, -1.0, 1.0)
        lx = int(round(sx * 255.0))
        ly = int(round(sy * 255.0))
        if lx != 0 and ly != 0:
            mode = "ackermann"
        elif ly > 0:
            mode = "forward"
        elif ly < 0:
            mode = "reverse"
        elif lx != 0:
            mode = "turn"
        else:
            mode = "stop"
        return lx, ly, 0, mode

    def publisher_loop(self):
        period = 1.0 / self.publish_rate
        last_mode = ""
        last_log = 0.0
        while rclpy.ok():
            now = time.monotonic()
            with self.lock:
                nx = self.state["nx"]
                ny = self.state["ny"]
                spin_left = self.state["spin_left"]
                spin_right = self.state["spin_right"]
                estop = self.state["estop"]
                clients = self.state["clients"]
                last_input = self.state["last_input"]
                last_disconnect = self.state["last_disconnect"]
                relax_pending = self.state["relax_pending"]
                rearm_pending = self.state["rearm_pending"]
                calibrate_pending = self.state["calibrate_pending"]
                auto_takeover = self.state["auto_takeover"]
                self.state["relax_pending"] = False
                self.state["rearm_pending"] = False
                self.state["calibrate_pending"] = False

            stale = (now - last_input) > self.deadman_timeout
            in_grace = (now - last_disconnect) < self.idle_grace_period
            lx, ly, rx, mode = self.compute_sticks(
                nx, ny, spin_left, spin_right, estop, stale
            )

            publishing = estop or clients > 0 or in_grace
            if publishing:
                message = ManualControl()
                message.lx = lx
                message.ly = ly
                message.rx = rx
                message.ry = 0
                message.relax = relax_pending
                message.rearm = rearm_pending
                message.boost = False
                message.calibrate = calibrate_pending
                message.auto_takeover = auto_takeover
                self.publisher.publish(message)

            display_mode = mode if publishing else "idle"

            with self.lock:
                self.state["last_published"] = {
                    "lx": lx, "ly": ly, "rx": rx, "mode": display_mode, "estop": estop,
                    "auto": auto_takeover,
                }

            if display_mode != last_mode or now - last_log > 1.0:
                print(
                    "mode=%s lx=%d ly=%d rx=%d clients=%d"
                    % (display_mode, lx, ly, rx, clients)
                )
                last_mode = display_mode
                last_log = now

            time.sleep(period)

    def handle_message(self, text):
        try:
            data = json.loads(text)
        except (ValueError, TypeError):
            return
        if not isinstance(data, dict):
            return
        kind = data.get("t")
        with self.lock:
            if kind == "joy":
                try:
                    x = float(data.get("x", 0.0))
                    y = float(data.get("y", 0.0))
                except (TypeError, ValueError):
                    return
                if not (math.isfinite(x) and math.isfinite(y)):
                    return
                self.state["last_input"] = time.monotonic()
                x = _clamp(x, -1.0, 1.0)
                y = _clamp(y, -1.0, 1.0)
                self.state["nx"] = x
                self.state["ny"] = y
            elif kind == "spin":
                self.state["last_input"] = time.monotonic()
                direction = data.get("dir")
                on = bool(data.get("on"))
                if direction == "left":
                    self.state["spin_left"] = on
                elif direction == "right":
                    self.state["spin_right"] = on
            elif kind == "estop":
                self.state["last_input"] = time.monotonic()
                self.state["estop"] = True
                self.state["nx"] = 0.0
                self.state["ny"] = 0.0
                self.state["spin_left"] = False
                self.state["spin_right"] = False
                self.state["auto_takeover"] = False
            elif kind == "arm":
                self.state["last_input"] = time.monotonic()
                self.state["estop"] = False
                self.state["nx"] = 0.0
                self.state["ny"] = 0.0
                self.state["spin_left"] = False
                self.state["spin_right"] = False
            elif kind == "relax":
                self.state["last_input"] = time.monotonic()
                self.state["relax_pending"] = True
            elif kind == "rearm":
                self.state["last_input"] = time.monotonic()
                self.state["rearm_pending"] = True
            elif kind == "calibrate":
                self.state["last_input"] = time.monotonic()
                self.state["calibrate_pending"] = True
            elif kind == "toggle_auto":
                self.state["last_input"] = time.monotonic()
                self.state["auto_takeover"] = not self.state["auto_takeover"]
            elif kind == "heartbeat":
                self.state["last_input"] = time.monotonic()

    # ---------------- web ----------------

    def _register_routes(self):
        app = self.app

        @app.get("/")
        async def index():
            return HTMLResponse(HTML_PAGE)

        @app.websocket("/ws")
        async def ws_endpoint(websocket: WebSocket):
            supplied_token = websocket.query_params.get("token", "")
            if self.access_token and not secrets.compare_digest(
                supplied_token, self.access_token
            ):
                await websocket.close(code=1008, reason="invalid access token")
                return
            await websocket.accept()
            self.connections.append(websocket)
            with self.lock:
                self.state["clients"] += 1
                self.state["last_input"] = time.monotonic()
            try:
                while True:
                    text = await websocket.receive_text()
                    self.handle_message(text)
            except WebSocketDisconnect:
                pass
            except Exception:
                pass
            finally:
                if websocket in self.connections:
                    self.connections.remove(websocket)
                with self.lock:
                    self.state["clients"] = max(0, self.state["clients"] - 1)
                    self.state["last_disconnect"] = time.monotonic()
                    self.state["nx"] = 0.0
                    self.state["ny"] = 0.0
                    self.state["spin_left"] = False
                    self.state["spin_right"] = False

    @asynccontextmanager
    async def _lifespan(self, app):
        task = asyncio.create_task(self._status_broadcaster())
        try:
            yield
        finally:
            task.cancel()

    async def _status_broadcaster(self):
        while True:
            await asyncio.sleep(0.5)
            with self.lock:
                payload = dict(self.state["last_published"])
            text = json.dumps({"t": "status", **payload})
            for websocket in list(self.connections):
                try:
                    await websocket.send_text(text)
                except Exception:
                    pass


def _local_ipv4_addresses():
    addresses = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127.") and ip not in addresses:
                addresses.append(ip)
    except OSError:
        pass
    return addresses


def main():
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    rclpy.init()
    server = WebJoystickServer()

    publisher_thread = threading.Thread(target=server.publisher_loop, daemon=True)
    publisher_thread.start()

    spin_thread = threading.Thread(target=lambda: rclpy.spin(server.node), daemon=True)
    spin_thread.start()

    print("Web joystick server 已启动，发布话题: " + server.command_topic)
    for ip in _local_ipv4_addresses():
        suffix = f"?token={server.access_token}" if server.access_token else ""
        print(f"  手机/笔记本打开: http://{ip}:{server.port}/{suffix}")

    try:
        uvicorn.run(server.app, host="0.0.0.0", port=server.port, log_level="info")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
