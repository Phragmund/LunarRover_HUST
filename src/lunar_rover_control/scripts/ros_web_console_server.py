#!/usr/bin/env python3
"""ROS bridge for the unified rover web console."""

from __future__ import annotations

import asyncio
import base64
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from lunar_rover_msgs.msg import TargetObservation
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Empty, String

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse, PlainTextResponse
except ImportError as exc:  # pragma: no cover
    sys.stderr.write(
        "ros_web_console_server 缺少 Web 依赖：pip3 install --user fastapi 'uvicorn[standard]'\n"
    )
    raise SystemExit(1) from exc


def default_console_path() -> str:
    here = Path(__file__).resolve()
    root = here.parents[3]
    return str(
        root
        / "src"
        / "lunar_rover_hardware"
        / "ubuntu_upper_dev_handoff_20260820"
        / "tools"
        / "web"
        / "rover_web_controller.html"
    )


def host_ip() -> str:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except OSError:
        return "127.0.0.1"


class RosWebConsoleServer(Node):
    def __init__(self):
        super().__init__("ros_web_console_server")
        self.port = int(self.declare_parameter("port", 8088).value)
        self.console_path = Path(
            str(self.declare_parameter("console_path", default_console_path()).value)
        )
        self.raw_topic = str(
            self.declare_parameter("raw_command_topic", "/h7/raw_command").value
        )
        self.raw_feedback_topic = str(
            self.declare_parameter("raw_feedback_topic", "/h7/rx_frame").value
        )
        self.camera_topic = str(
            self.declare_parameter(
                "camera_topic", "/perception/debug/fusion_overlay/compressed"
            ).value
        )
        self.max_camera_hz = float(self.declare_parameter("max_camera_hz", 5.0).value)
        self.access_token = str(self.declare_parameter("access_token", "").value)
        self.full_chain_start_command = str(self.declare_parameter(
            "full_chain_start_command",
            "/home/phragmund/LunarRover/scripts/start_full_real_nav2_chain.sh",
        ).value)

        self.loop = None
        self.connections = set()
        self.connections_lock = threading.Lock()
        self.last_camera_sent = 0.0
        self.last_grasp_status = "ABORT: 未启动，等待开始搜索"
        self.last_grasp_mode = "idle"
        self.last_workspace_status = "NO_OBSERVATION"
        self.last_pose_average_status = "NO_SAMPLES"
        self.last_handeye_status = {
            "state": "stopped",
            "samples": 0,
            "output_dir": "",
        }
        self.handeye_process = None
        self.handeye_lock = threading.Lock()
        self.full_chain_process = None
        self.full_chain_lock = threading.Lock()
        self.full_chain_status = {
            "state": "idle",
            "detail": "未启动完整链路。",
        }
        self.last_target_observation = None
        self.last_target_pose = None

        self.raw_publisher = self.create_publisher(String, self.raw_topic, 10)
        self.grasp_command_publisher = self.create_publisher(
            String, "/grasp/command", 10
        )
        self.grasp_trigger_publisher = self.create_publisher(Empty, "/grasp/trigger", 10)
        self.handeye_command_publisher = self.create_publisher(
            String, "/handeye/calibration_command", 10
        )
        camera_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CompressedImage, self.camera_topic, self.on_camera, camera_qos)
        self.create_subscription(String, self.raw_feedback_topic, self.on_h7_frame, 50)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/grasp/status", self.on_grasp_status, state_qos)
        self.create_subscription(String, "/perception/grasp_mode", self.on_grasp_mode, state_qos)
        self.create_subscription(String, "/grasp/workspace_status", self.on_workspace_status, state_qos)
        self.create_subscription(
            String, "/grasp/pose_average_status", self.on_pose_average_status, state_qos
        )
        self.create_subscription(
            String,
            "/handeye/calibration_status",
            self.on_handeye_status,
            state_qos,
        )
        self.create_subscription(
            TargetObservation,
            "/perception/target_observation",
            self.on_target_observation,
            10,
        )
        self.create_subscription(PoseStamped, "/grasp/target_pose", self.on_target_pose, 10)

        self.app = FastAPI()
        self._register_routes()

    def _register_routes(self):
        @self.app.get("/")
        async def index():
            try:
                html = self.console_path.read_text(encoding="utf-8")
            except OSError as exc:
                return PlainTextResponse(
                    f"Cannot read console HTML: {self.console_path}\n{exc}",
                    status_code=500,
                )
            return HTMLResponse(html)

        @self.app.websocket("/ros")
        async def websocket_endpoint(websocket: WebSocket):
            if self.access_token:
                token = websocket.query_params.get("token", "")
                if token != self.access_token:
                    await websocket.close(code=1008)
                    return
            await websocket.accept()
            with self.connections_lock:
                self.connections.add(websocket)
            await self.send_one(
                websocket,
                {
                    "type": "ros_status",
                    "connected": True,
                    "raw_topic": self.raw_topic,
                    "raw_feedback_topic": self.raw_feedback_topic,
                    "camera_topic": self.camera_topic,
                },
            )
            await self._send_cached_state(websocket)
            try:
                while True:
                    data = await websocket.receive_text()
                    self.handle_client_message(data)
            except WebSocketDisconnect:
                pass
            finally:
                with self.connections_lock:
                    self.connections.discard(websocket)

    def handle_client_message(self, data: str):
        try:
            message = json.loads(data)
        except json.JSONDecodeError:
            return
        if message.get("type") == "raw":
            frame = str(message.get("frame", "")).strip()
            if not frame:
                return
            command = String()
            command.data = frame
            self.raw_publisher.publish(command)
            self.broadcast_threadsafe({"type": "console_log", "text": f"ROS TX {frame}"})
        elif message.get("type") == "virtual_key":
            # A phone is only a keyboard proxy.  The browser that owns the
            # unified-console IK state consumes this event and emits its usual
            # command frames; this bridge never synthesizes an ARM frame.
            key = str(message.get("key", "")).lower()
            pressed = message.get("pressed") is True
            if key in {
                "w", "s", "a", "d", " ", "shift", "arrowup", "arrowdown",
                "arrowleft", "arrowright", "r", "f", "e",
            }:
                self.broadcast_threadsafe(
                    {"type": "virtual_key", "key": key, "pressed": pressed}
                )
        elif message.get("type") == "grasp_command":
            command_text = str(message.get("command", "")).strip()
            if not command_text:
                return
            command = String()
            command.data = command_text
            self.grasp_command_publisher.publish(command)
            self.broadcast_threadsafe(
                {"type": "console_log", "text": f"ROS GRASP {command_text}"}
            )
        elif message.get("type") == "handeye_command":
            command = str(message.get("command", "")).strip().lower()
            if command == "start":
                self.start_handeye_sampler()
            elif command in {"p", "q"}:
                self.send_handeye_command(command)
        elif message.get("type") == "start_full_nav2_chain":
            requested = message.get("start_mapping", False)
            start_mapping = requested is True or str(requested).lower() in {
                "1", "true", "yes", "on"
            }
            self.start_full_nav2_chain(start_mapping)
        elif message.get("type") == "ping":
            self.broadcast_threadsafe({"type": "ros_status", "connected": True})

    async def send_one(self, websocket: WebSocket, payload: dict):
        await websocket.send_text(json.dumps(payload, ensure_ascii=False))

    async def _send_cached_state(self, websocket: WebSocket):
        await self.send_one(
            websocket, {"type": "grasp_status", "text": self.last_grasp_status}
        )
        await self.send_one(
            websocket, {"type": "grasp_mode", "text": self.last_grasp_mode}
        )
        await self.send_one(
            websocket,
            {"type": "workspace_status", "text": self.last_workspace_status},
        )
        await self.send_one(
            websocket,
            {"type": "pose_average_status", "text": self.last_pose_average_status},
        )
        await self.send_one(
            websocket,
            {"type": "handeye_status", **self.last_handeye_status},
        )
        await self.send_one(
            websocket, {"type": "full_chain_status", **self.full_chain_status}
        )
        if self.last_target_observation is not None:
            await self.send_one(websocket, self.last_target_observation)
        if self.last_target_pose is not None:
            await self.send_one(websocket, self.last_target_pose)

    async def broadcast(self, payload: dict):
        with self.connections_lock:
            sockets = list(self.connections)
        if not sockets:
            return
        data = json.dumps(payload, ensure_ascii=False)
        dead = []
        for websocket in sockets:
            try:
                await websocket.send_text(data)
            except Exception:  # noqa: BLE001
                dead.append(websocket)
        if dead:
            with self.connections_lock:
                for websocket in dead:
                    self.connections.discard(websocket)

    def broadcast_threadsafe(self, payload: dict):
        if self.loop is None:
            return
        asyncio.run_coroutine_threadsafe(self.broadcast(payload), self.loop)

    def on_camera(self, message: CompressedImage):
        now = time.monotonic()
        if self.max_camera_hz > 0.0 and now - self.last_camera_sent < 1.0 / self.max_camera_hz:
            return
        self.last_camera_sent = now
        self.broadcast_threadsafe(
            {
                "type": "camera",
                "format": message.format or "jpeg",
                "stamp": {
                    "sec": int(message.header.stamp.sec),
                    "nanosec": int(message.header.stamp.nanosec),
                },
                "data": base64.b64encode(bytes(message.data)).decode("ascii"),
            }
        )

    def on_h7_frame(self, message: String):
        self.broadcast_threadsafe({"type": "h7_frame", "frame": message.data})

    def on_joint_state(self, message: JointState):
        self.broadcast_threadsafe(
            {
                "type": "joint_state",
                "names": list(message.name),
                "positions": [float(value) for value in message.position],
            }
        )

    def on_grasp_status(self, message: String):
        self.last_grasp_status = message.data or "--"
        self.broadcast_threadsafe({"type": "grasp_status", "text": self.last_grasp_status})

    def on_grasp_mode(self, message: String):
        self.last_grasp_mode = message.data or "--"
        self.broadcast_threadsafe({"type": "grasp_mode", "text": self.last_grasp_mode})

    def on_workspace_status(self, message: String):
        self.last_workspace_status = message.data or "--"
        self.broadcast_threadsafe(
            {"type": "workspace_status", "text": self.last_workspace_status}
        )

    def on_pose_average_status(self, message: String):
        self.last_pose_average_status = message.data or "--"
        self.broadcast_threadsafe(
            {"type": "pose_average_status", "text": self.last_pose_average_status}
        )

    def on_handeye_status(self, message: String):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                raise ValueError("status is not an object")
        except (TypeError, ValueError, json.JSONDecodeError):
            status = {"state": message.data or "unknown", "samples": 0, "output_dir": ""}
        self.last_handeye_status = {
            "state": str(status.get("state", "unknown")),
            "samples": int(status.get("samples", 0)),
            "output_dir": str(status.get("output_dir", "")),
        }
        self.broadcast_threadsafe({"type": "handeye_status", **self.last_handeye_status})

    def start_handeye_sampler(self):
        with self.handeye_lock:
            if self.handeye_process is not None and self.handeye_process.poll() is None:
                self.broadcast_threadsafe(
                    {"type": "console_log", "text": "ChArUco 采样节点已经运行"}
                )
                return
            root = Path(__file__).resolve().parents[3]
            command = ["ros2", "run", "lunar_rover_manipulation", "charuco_handeye_sampler_node.py"]
            try:
                self.handeye_process = subprocess.Popen(
                    command,
                    cwd=str(root),
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                threading.Thread(
                    target=self._watch_handeye_process,
                    args=(self.handeye_process,),
                    daemon=True,
                ).start()
            except OSError as exc:
                self.last_handeye_status = {
                    "state": "error",
                    "samples": 0,
                    "output_dir": "",
                }
                self.broadcast_threadsafe(
                    {"type": "handeye_status", **self.last_handeye_status}
                )
                self.broadcast_threadsafe(
                    {"type": "console_log", "text": f"启动 ChArUco 采样节点失败: {exc}"}
                )
                return
        self.last_handeye_status["state"] = "starting"
        self.broadcast_threadsafe({"type": "handeye_status", **self.last_handeye_status})
        self.broadcast_threadsafe(
            {"type": "console_log", "text": "已启动 ChArUco 手眼采样节点"}
        )

    def start_full_nav2_chain(self, start_mapping):
        """Start standby only, or optionally continue into the full Nav2 stack."""
        with self.full_chain_lock:
            if self.full_chain_process is not None and self.full_chain_process.poll() is None:
                self.broadcast_threadsafe(
                    {"type": "console_log", "text": "完整链路已经在启动中"}
                )
                return
            try:
                self.full_chain_process = subprocess.Popen(
                    ["bash", self.full_chain_start_command, "true" if start_mapping else "false"],
                    cwd=str(Path(__file__).resolve().parents[3]),
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
            except OSError as exc:
                self.full_chain_status = {"state": "failed", "detail": str(exc)}
                self.broadcast_threadsafe(
                    {"type": "full_chain_status", **self.full_chain_status}
                )
                return
            self.full_chain_status = {
                "state": "starting",
                "detail": (
                    "正在重启待命服务，并拉起视觉抓取、FAST-LIO / Nav2…"
                    if start_mapping
                    else "正在启动雷达、底盘、手机服务与视觉抓取；建图将由手机端发起…"
                ),
            }
            process = self.full_chain_process
        self.broadcast_threadsafe({"type": "full_chain_status", **self.full_chain_status})
        self.broadcast_threadsafe(
            {"type": "console_log", "text": "开始完整实车链路（导航 + 视觉抓取）；请保持车体静止"}
        )
        threading.Thread(
            target=self._watch_full_nav2_chain,
            args=(process, start_mapping), daemon=True
        ).start()

    def _watch_full_nav2_chain(self, process, start_mapping):
        result = process.wait()
        with self.full_chain_lock:
            if self.full_chain_process is not process:
                return
            self.full_chain_process = None
            self.full_chain_status = {
                "state": "ready" if result == 0 else "failed",
                "detail": (
                    "完整链路已启动：视觉抓取、手机遥控、建图、Nav2、探索和 RViz 均已提交。"
                    if result == 0 and start_mapping
                    else "待命与视觉抓取链路已启动：请在手机端点击“开始建图导航”。"
                    if result == 0
                    else "完整链路启动失败，请查看 /tmp/start_full_real_nav2_chain.log"
                ),
            }
        self.broadcast_threadsafe({"type": "full_chain_status", **self.full_chain_status})
        self.broadcast_threadsafe(
            {"type": "console_log", "text": self.full_chain_status["detail"]}
        )

    def _watch_handeye_process(self, process):
        process.wait()
        with self.handeye_lock:
            if self.handeye_process is not process:
                return
            self.handeye_process = None
        status = dict(self.last_handeye_status)
        status["state"] = "stopped"
        self.last_handeye_status = status
        self.broadcast_threadsafe({"type": "handeye_status", **status})

    def send_handeye_command(self, command):
        if command == "q":
            self.handeye_command_publisher.publish(String(data="q"))
            self.broadcast_threadsafe(
                {"type": "console_log", "text": "ChArUco 采样请求退出并保存"}
            )
            return
        with self.handeye_lock:
            running = self.handeye_process is not None and self.handeye_process.poll() is None
        if not running:
            self.broadcast_threadsafe(
                {"type": "console_log", "text": "请先启动 ChArUco 手眼采样"}
            )
            return
        self.handeye_command_publisher.publish(String(data=command))

    def stop_handeye_process(self):
        with self.handeye_lock:
            process = self.handeye_process
            self.handeye_process = None
        if process is None or process.poll() is not None:
            return
        self.handeye_command_publisher.publish(String(data="q"))
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except (OSError, ProcessLookupError):
                process.terminate()

    def on_target_observation(self, message: TargetObservation):
        p = message.pose.position
        d = message.dimensions
        payload = {
            "type": "target_observation",
            "class_id": message.class_id,
            "confidence": float(message.confidence),
            "frame_id": message.header.frame_id,
            "position": [float(p.x), float(p.y), float(p.z)],
            "dimensions": [float(d.x), float(d.y), float(d.z)],
            "surface_type": int(message.surface_type),
            "source": message.source,
        }
        self.last_target_observation = payload
        self.broadcast_threadsafe(payload)

    def on_target_pose(self, message: PoseStamped):
        p = message.pose.position
        payload = {
            "type": "target_pose",
            "frame_id": message.header.frame_id,
            "position": [float(p.x), float(p.y), float(p.z)],
        }
        self.last_target_pose = payload
        self.broadcast_threadsafe(payload)

    def run_web(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        config = uvicorn.Config(
            self.app,
            host="0.0.0.0",
            port=self.port,
            log_level="warning",
        )
        server = uvicorn.Server(config)
        self.get_logger().info(
            "ROS unified web console local: http://127.0.0.1:%d" % self.port
        )
        self.get_logger().info(
            "ROS unified web console LAN: http://%s:%d (ROS/camera only; browser Web Serial needs localhost or file://)"
            % (host_ip(), self.port)
        )
        self.loop.run_until_complete(server.serve())


def main():
    rclpy.init()
    node = RosWebConsoleServer()
    web_thread = threading.Thread(target=node.run_web, daemon=True)
    web_thread.start()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_handeye_process()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
