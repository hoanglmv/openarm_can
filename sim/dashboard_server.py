#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
OpenArm Dashboard Server orchestrator.
Manages WebSocket streaming, HTTP server, 400Hz trajectory generator,
external UDP teleop stream receiver, and USB hotplug monitoring.
"""

import asyncio
import json
import math
import os
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from typing import Dict, List, Optional, Set

import websockets

try:
    from config import (
        CAMERA_ALIGNED_DEPTH_TOPIC,
        CAMERA_DEPTH_TOPIC,
        CAMERA_RAW_DEPTH_TOPIC,
        CAMERA_RAW_RGB_TOPIC,
        CAMERA_STREAM_PORT,
        CAMERA_TOPIC,
        CONTROL_FREQ,
        DATA_FREQUENCY_HZ,
        HTTP_PORT,
        JOINT_BRIDGE_PORT,
        JOINT_LIMITS,
        JOINT_NAME_TO_ID,
        JOINT_NAMES,
        TELEMETRY_FREQ,
        UDP_EXPORT_PORT,
        UDP_STREAM_PORT,
        WS_PORT,
        double_to_uint,
    )
    from can_bridge import RealRobotHardwareBridge
    from exporter import JointStateExporter100Hz
    from http_server import CustomHTTPHandler
    from motor_simulator import DamiaoArmSimulator
except ImportError:
    from .config import (
        CAMERA_ALIGNED_DEPTH_TOPIC,
        CAMERA_DEPTH_TOPIC,
        CAMERA_RAW_DEPTH_TOPIC,
        CAMERA_RAW_RGB_TOPIC,
        CAMERA_STREAM_PORT,
        CAMERA_TOPIC,
        CONTROL_FREQ,
        DATA_FREQUENCY_HZ,
        HTTP_PORT,
        JOINT_BRIDGE_PORT,
        JOINT_LIMITS,
        JOINT_NAME_TO_ID,
        JOINT_NAMES,
        TELEMETRY_FREQ,
        UDP_EXPORT_PORT,
        UDP_STREAM_PORT,
        WS_PORT,
        double_to_uint,
    )
    from .can_bridge import RealRobotHardwareBridge
    from .exporter import JointStateExporter100Hz
    from .http_server import CustomHTTPHandler
    from .motor_simulator import DamiaoArmSimulator


class OpenArmDashboardServer:
    """Core server orchestrating hardware/simulator, web server, and realtime comms."""

    def __init__(self, mode: str = "real", can0_if: str = "can0", can1_if: Optional[str] = "can1"):
        self.mode = mode
        self.can0_if = can0_if
        self.can1_if = can1_if
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.running = True
        self.velocity_limit = 0.25  # rad/s (~14°/s) gentle & safe velocity limit
        self.gripper_invert = {8: False, 16: False}  # Direction invert flag (default False: 0mm=closed/0.0rad, 41.5mm=open/1.15rad)

        if self.mode == "real":
            print(f"[Dashboard] Initializing in REAL ROBOT HARDWARE MODE on {can0_if} / {can1_if}")
            self.hw = RealRobotHardwareBridge(can0_if, can1_if)
            self.motors = self.hw.motors
        else:
            print(f"[Dashboard] Initializing in SIMULATION MODE on vcan0")
            self.hw = DamiaoArmSimulator("vcan0")
            self.motors = self.hw.motors

        # Initialize smooth command states
        for m in self.motors.values():
            m.q_target = 0.0
            m.q_cmd = 0.0
            m.kp = 18.0
            m.kd = 2.0

        # Camera & ACT ROS 2 Bridge processes
        self.camera_process = None
        self.camera_driver_process = None
        self.rgbd_process = None
        self.joint_bridge_process = None
        self.joint_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # External joint state streaming diagnostics & metrics
        self.stream_stats = {
            "packets": 0,
            "last_time": 0.0,
            "hz": 0.0,
            "source": "None"
        }

        # Continuous 100Hz Joint State Exporter (Activated upon robot connection)
        export_dir_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "exports")
        self.exporter = JointStateExporter100Hz(self, export_dir=export_dir_path, udp_port=UDP_EXPORT_PORT)
        self.loop = None

    def start(self):
        """Start all background services, threads, and WebSocket server."""
        # 1. Start camera & ROS 2 bridges
        self._start_camera_driver()
        self._start_rgbd_preprocessor()
        self._start_camera_bridge()
        self._start_joint_bridge()

        # 2. Start hardware bridge or simulator
        self.hw.start()

        # 3. Start smooth trajectory generator thread (400 Hz)
        self.traj_thread = threading.Thread(target=self._trajectory_loop, daemon=True)
        self.traj_thread.start()

        # 4. Start 50Hz UDP joint stream for ACT data recorder & ROS bridge
        self.joint_stream_thread = threading.Thread(
            target=self._joint_stream_loop,
            daemon=True,
        )
        self.joint_stream_thread.start()

        # 5. Start auto-hotplug interface monitor
        self.hotplug_thread = threading.Thread(target=self._hotplug_monitor_loop, daemon=True)
        self.hotplug_thread.start()

        # 6. Start HTTP server thread
        ThreadingHTTPServer.allow_reuse_address = True
        self.httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), CustomHTTPHandler)
        self.httpd.app = self
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        print(f"[Dashboard] HTTP Server running on http://localhost:{HTTP_PORT} (REST API: /api/joint_state, /api/export/*)")

        # 7. Start high-speed UDP Joint Stream receiver (port 9870 for zero-latency ROS 2 / teleop streams)
        self.udp_thread = threading.Thread(target=self._udp_receiver_loop, daemon=True)
        self.udp_thread.start()

        # 8. Start continuous 100Hz Joint State Exporter thread
        self.export_thread = threading.Thread(target=self.exporter.loop, daemon=True)
        self.export_thread.start()
        if self.mode == "real":
            self.exporter.start_session("robot_boot")

        # 9. Start WebSocket & Telemetry Broadcaster
        asyncio.run(self.run_ws_server())

    def _start_camera_driver(self):
        """Launch Intel RealSense ROS 2 camera driver if available."""
        if "--no-camera" in sys.argv:
            print("[Camera] Driver disabled by --no-camera")
            return

        ros2 = shutil.which("ros2")
        if not ros2:
            print("[Camera] ros2 CLI not found; skipping RealSense launch")
            return

        try:
            subprocess.run(
                ["ros2", "node", "list"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=1.5,
                check=False,
            )
        except Exception:
            pass

        try:
            color_profile = os.environ.get(
                "OPENARM_CAMERA_COLOR_PROFILE", "640x480x60"
            )
            depth_profile = os.environ.get(
                "OPENARM_CAMERA_DEPTH_PROFILE", "640x480x60"
            )
            self.camera_driver_process = subprocess.Popen(
                [
                    ros2,
                    "launch",
                    "realsense2_camera",
                    "rs_launch.py",
                    "enable_color:=true",
                    "enable_depth:=true",
                    "enable_sync:=true",
                    "align_depth.enable:=true",
                    "rgb_camera.color_profile:=" + color_profile,
                    "depth_module.depth_profile:=" + depth_profile,
                ],
            )
            print(
                "[Camera] Starting Intel RealSense RGB + aligned depth "
                f"(color={color_profile}, depth={depth_profile})"
            )
        except Exception as error:
            print(f"[Camera] Could not start RealSense driver: {error}")

    def _start_rgbd_preprocessor(self):
        """Launch synchronized ACT RGB-D output preprocessor (640x480 @ 50 Hz)."""
        if "--no-camera" in sys.argv:
            return

        script = os.path.join(os.path.dirname(__file__), "rgbd_preprocessor.py")
        camera_python = os.environ.get("OPENARM_CAMERA_PYTHON", "/usr/bin/python3")
        if not os.path.exists(camera_python):
            camera_python = shutil.which("python3") or sys.executable

        try:
            self.rgbd_process = subprocess.Popen(
                [
                    camera_python,
                    script,
                    "--rgb-input",
                    CAMERA_RAW_RGB_TOPIC,
                    "--depth-input",
                    CAMERA_ALIGNED_DEPTH_TOPIC,
                    "--rgb-output",
                    CAMERA_TOPIC,
                    "--depth-output",
                    CAMERA_DEPTH_TOPIC,
                    "--width",
                    "640",
                    "--height",
                    "480",
                    "--rate",
                    str(int(DATA_FREQUENCY_HZ)),
                ],
            )
            print(
                f"[Camera] Starting synchronized ACT RGB-D output (640x480 @ {int(DATA_FREQUENCY_HZ)} Hz)"
            )
        except Exception as error:
            print(f"[Camera] Could not start RGB-D preprocessor: {error}")

    def _start_camera_bridge(self):
        """Launch ROS image bridge to HTTP MJPEG stream (port 8890)."""
        if "--no-camera" in sys.argv:
            print("[Camera] Bridge disabled by --no-camera")
            return

        bridge_script = os.path.join(os.path.dirname(__file__), "camera_stream.py")
        camera_python = os.environ.get("OPENARM_CAMERA_PYTHON", "/usr/bin/python3")
        if not os.path.exists(camera_python):
            camera_python = shutil.which("python3") or sys.executable

        try:
            self.camera_process = subprocess.Popen(
                [
                    camera_python,
                    bridge_script,
                    "--topic",
                    CAMERA_TOPIC,
                    "--port",
                    str(CAMERA_STREAM_PORT),
                ],
            )
            print(
                f"[Camera] Starting ROS image bridge for {CAMERA_TOPIC} "
                f"on port {CAMERA_STREAM_PORT}"
            )
        except Exception as error:
            print(f"[Camera] Could not start ROS image bridge: {error}")

    def _start_joint_bridge(self):
        """Launch OpenArm ROS 2 joint bridge node."""
        script = os.path.join(os.path.dirname(__file__), "openarm_joint_bridge.py")
        ros_python = os.environ.get("OPENARM_CAMERA_PYTHON", "/usr/bin/python3")
        if not os.path.exists(ros_python):
            ros_python = shutil.which("python3") or sys.executable
        try:
            self.joint_bridge_process = subprocess.Popen(
                [ros_python, script, "--port", str(JOINT_BRIDGE_PORT)],
            )
            print(
                f"[ROS] Publishing /openarm/joint_states and /openarm/joint_commands at {int(DATA_FREQUENCY_HZ)} Hz"
            )
        except Exception as error:
            print(f"[ROS] Could not start OpenArm joint bridge: {error}")

    def _joint_stream_loop(self):
        """Stream joint states and commands over UDP to the ROS 2 joint bridge at 50 Hz."""
        period = 1.0 / DATA_FREQUENCY_HZ
        next_tick = time.perf_counter()
        while self.running:
            try:
                with self.hw.lock:
                    qpos = []
                    qvel = []
                    effort = []
                    action = []
                    for motor_id in range(1, 17):
                        motor = self.motors[motor_id]
                        # The ACT dataset stores all 16 motor axes, including
                        # both gripper motors, in their native angular units.
                        qpos.append(float(motor.q))
                        qvel.append(float(motor.dq))
                        effort.append(float(motor.tau))
                        action.append(float(motor.q_target))
                payload = json.dumps(
                    {
                        "timestamp_ns": time.time_ns(),
                        "names": JOINT_NAMES,
                        "qpos": qpos,
                        "qvel": qvel,
                        "effort": effort,
                        "action": action,
                    },
                    separators=(",", ":"),
                ).encode("utf-8")
                self.joint_udp_socket.sendto(
                    payload,
                    ("127.0.0.1", JOINT_BRIDGE_PORT),
                )
            except Exception:
                pass

            next_tick += period
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            elif sleep_time < -period:
                next_tick = time.perf_counter()

    def stop(self):
        """Clean shutdown of all background threads, processes, and sockets."""
        self.running = False
        try:
            self.hw.stop()
        except Exception:
            pass
        if getattr(self, "httpd", None):
            try:
                self.httpd.shutdown()
                self.httpd.server_close()
            except Exception:
                pass
        try:
            self.joint_udp_socket.close()
        except Exception:
            pass
        if self.camera_process and self.camera_process.poll() is None:
            self.camera_process.terminate()
            try:
                self.camera_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.camera_process.kill()
        if self.rgbd_process and self.rgbd_process.poll() is None:
            self.rgbd_process.terminate()
            try:
                self.rgbd_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.rgbd_process.kill()
        if self.joint_bridge_process and self.joint_bridge_process.poll() is None:
            self.joint_bridge_process.terminate()
            try:
                self.joint_bridge_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.joint_bridge_process.kill()
        if self.camera_driver_process and self.camera_driver_process.poll() is None:
            self.camera_driver_process.terminate()
            try:
                self.camera_driver_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.camera_driver_process.kill()

    def _find_can_usb(self):
        """Query usbipd on Windows host for connected CAN adapters."""
        try:
            res = subprocess.run(["usbipd", "state"], capture_output=True, text=True, timeout=3)
            if res.returncode == 0 and res.stdout.strip():
                data = json.loads(res.stdout)
                devices = data.get("Devices", [])
                for d in devices:
                    desc = d.get("Description", "") or ""
                    inst = d.get("InstanceId", "") or ""
                    busid = d.get("BusId")
                    if ('PCAN' in desc.upper() or '0C72' in inst.upper() or 'CAN' in desc.upper()) and busid:
                        return str(busid), desc
        except Exception as e:
            print("[USB Search Error]:", e)

        try:
            res = subprocess.run(["usbipd", "list"], capture_output=True, text=True, timeout=3)
            for line in res.stdout.splitlines():
                if "PCAN" in line.upper() or "0C72:0011" in line.lower():
                    parts = line.split()
                    if parts and '-' in parts[0]:
                        return parts[0], "PCAN-USB Pro FD"
        except Exception:
            pass

        return None, None

    def _exec_connect_usb(self):
        """Execute automated USB attach and CAN-FD configuration sequence."""
        try:
            print("[USB] Bắt đầu kết nối USB Robot...")
            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("info", "Đang quét cổng USB Robot..."), self.loop)

            can0_present = os.path.exists("/sys/class/net/can0")
            busid = None
            desc = "PCAN-USB Pro FD"

            if not can0_present:
                busid, found_desc = self._find_can_usb()
                if found_desc:
                    desc = found_desc
                if not busid:
                    msg = "Chưa phát hiện thiết bị USB CAN cắm trên máy tính. Vui lòng cắm cáp USB nối đến robot."
                    print(f"[USB Error] {msg}")
                    if hasattr(self, 'loop') and self.loop:
                        asyncio.run_coroutine_threadsafe(self.broadcast_notice("error", msg), self.loop)
                    return

                print(f"[USB] Đang gắn thiết bị USB (BusID: {busid}, {desc}) vào WSL2...")
                if hasattr(self, 'loop') and self.loop:
                    asyncio.run_coroutine_threadsafe(self.broadcast_notice("info", f"Đang gắn USB {busid} ({desc}) vào WSL2..."), self.loop)

                # Attach via usbipd
                att_res = subprocess.run(["usbipd", "attach", "--wsl", "--busid", busid], capture_output=True, text=True, timeout=8)
                if att_res.returncode != 0 and "already attached" not in att_res.stderr.lower() and "already attached" not in att_res.stdout.lower():
                    print(f"[USB] Thử detach rồi attach lại: {att_res.stderr.strip() or att_res.stdout.strip()}")
                    subprocess.run(["usbipd", "detach", "--busid", busid], capture_output=True, text=True, timeout=5)
                    time.sleep(0.5)
                    subprocess.run(["usbipd", "attach", "--wsl", "--busid", busid], capture_output=True, text=True, timeout=8)

                # Load required kernel modules
                for mod in ["vhci-hcd", "can", "can-raw", "can-dev", "peak_usb"]:
                    subprocess.run(["sudo", "modprobe", mod], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

                # Wait for can0 to appear in /sys/class/net
                for _ in range(12):
                    if os.path.exists("/sys/class/net/can0"):
                        break
                    time.sleep(0.3)

            # Bring up CAN-FD on can0 and can1
            configured_any = False
            for iface in ["can0", "can1"]:
                if os.path.exists(f"/sys/class/net/{iface}"):
                    subprocess.run(["sudo", "ip", "link", "set", iface, "down"], check=False)
                    cmd = [
                        "sudo", "ip", "link", "set", iface, "type", "can",
                        "bitrate", "1000000", "sample-point", "0.75",
                        "dbitrate", "5000000", "dsample-point", "0.75",
                        "dsjw", "2", "fd", "on"
                    ]
                    r = subprocess.run(cmd, check=False)
                    if r.returncode != 0:
                        subprocess.run(["sudo", "ip", "link", "set", iface, "type", "can", "bitrate", "1000000"], check=False)
                    subprocess.run(["sudo", "ip", "link", "set", iface, "up"], check=False)
                    subprocess.run(["sudo", "ip", "link", "set", iface, "txqueuelen", "1000"], check=False)
                    configured_any = True

            if configured_any:
                time.sleep(0.3)
                # ALWAYS recreate hardware bridge so fresh SocketCAN file descriptors are bound
                try:
                    self.hw.stop()
                    new_hw = RealRobotHardwareBridge(self.can0_if, self.can1_if)
                    new_hw.start()
                    self.hw = new_hw
                    self.motors = self.hw.motors
                    self.mode = "real"
                    print(f"[USB] Kết nối thành công! Đang ở chế độ REAL ROBOT HARDWARE MODE ({self.can0_if}/{self.can1_if})")
                    self.hw.query_all_physical()
                    self.exporter.start_session("usb_connected")
                except Exception as e:
                    print(f"[USB Switch Error]: {e}")

                dev_info = f" ({desc}, Bus: {busid})" if busid else " (can0/can1)"
                msg = f"Đã kết nối thành công Robot thật qua USB{dev_info}!"
                if hasattr(self, 'loop') and self.loop:
                    asyncio.run_coroutine_threadsafe(self.broadcast_notice("success", msg), self.loop)
            else:
                msg = "Không tìm thấy interface can0/can1 sau khi gắn USB. Vui lòng kiểm tra nguồn và cáp robot."
                print(f"[USB Warning] {msg}")
                if hasattr(self, 'loop') and self.loop:
                    asyncio.run_coroutine_threadsafe(self.broadcast_notice("warning", msg), self.loop)

        except Exception as e:
            err_msg = f"Lỗi trong quá trình kết nối USB: {e}"
            print(f"[USB Exception] {err_msg}")
            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("error", err_msg), self.loop)

    def _exec_disconnect_usb(self):
        """Safely disarm robot, tear down CAN interfaces, and detach USB adapter."""
        try:
            print("[USB] Ngắt kết nối USB Robot...")
            self.exporter.close_session()

            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("info", "Đang ngắt kết nối USB và ngắt torque an toàn..."), self.loop)

            # 1. Disarm all motors safely first
            with self.hw.lock:
                for m in self.motors.values():
                    m.enabled = False
                    m.error_code = 0
            if self.mode == "real":
                for m in self.motors.values():
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFD]))

            time.sleep(0.3)

            # 2. Down CAN interfaces
            for iface in ["can0", "can1"]:
                if os.path.exists(f"/sys/class/net/{iface}"):
                    subprocess.run(["sudo", "ip", "link", "set", iface, "down"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # 3. Detach via usbipd
            busid, _ = self._find_can_usb()
            if busid:
                subprocess.run(["usbipd", "detach", "--busid", busid], check=False, timeout=5)

            # 4. Fallback to SIM
            try:
                self.hw.stop()
                new_sim = DamiaoArmSimulator("vcan0")
                new_sim.start()
                self.hw = new_sim
                self.motors = self.hw.motors
                self.mode = "sim"
                print("[USB] Đã ngắt kết nối robot thật. Chuyển sang SIMULATION MODE (vcan0)")
            except Exception as e:
                print(f"[USB Fallback Error]: {e}")

            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("warning", "Đã ngắt kết nối USB Robot. Hệ thống đang ở chế độ Mô phỏng (SIM)."), self.loop)
        except Exception as e:
            print(f"[USB Disconnect Error]: {e}")
            if hasattr(self, 'loop') and self.loop:
                asyncio.run_coroutine_threadsafe(self.broadcast_notice("error", f"Lỗi ngắt kết nối: {e}"), self.loop)

    async def broadcast_notice(self, level: str, message: str):
        """Send notification alert modal/toast to all connected web clients."""
        msg = json.dumps({
            "type": "notice",
            "level": level,
            "message": message,
            "mode": self.mode
        })
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except Exception:
                pass

    def _hotplug_monitor_loop(self):
        """Continuously check for physical CAN hotplug (can0/can1) without needing restart."""
        while self.running:
            time.sleep(2.0)
            can0_present = os.path.exists("/sys/class/net/can0")
            if not can0_present:
                # Try auto-detecting and attaching USB adapter if host has it plugged in
                busid, found_desc = self._find_can_usb()
                if busid:
                    try:
                        print(f"[Hotplug] Phát hiện USB PCAN trên Windows host ({busid}). Đang tự động gắn vào WSL2...")
                        subprocess.run(["usbipd", "attach", "--wsl", "--busid", busid], capture_output=True, text=True, timeout=8)
                        for mod in ["vhci-hcd", "can", "can-raw", "can-dev", "peak_usb"]:
                            subprocess.run(["sudo", "modprobe", mod], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        time.sleep(0.5)
                        can0_present = os.path.exists("/sys/class/net/can0")
                    except Exception:
                        pass

            if self.mode == "sim" and can0_present:
                print("[Hotplug] Detected physical can0 interface! Switching to REAL ROBOT HARDWARE MODE...")
                try:
                    for iface in ["can0", "can1"]:
                        if os.path.exists(f"/sys/class/net/{iface}"):
                            subprocess.run(["sudo", "ip", "link", "set", iface, "type", "can",
                                            "bitrate", "1000000", "sample-point", "0.75",
                                            "dbitrate", "5000000", "dsample-point", "0.75",
                                            "dsjw", "2", "fd", "on"], check=False)
                            subprocess.run(["sudo", "ip", "link", "set", iface, "up"], check=False)
                            subprocess.run(["sudo", "ip", "link", "set", iface, "txqueuelen", "1000"], check=False)
                    new_hw = RealRobotHardwareBridge(self.can0_if, self.can1_if)
                    new_hw.start()
                    self.hw.stop()
                    self.hw = new_hw
                    self.motors = self.hw.motors
                    self.mode = "real"
                    print(f"[Hotplug] Switched to REAL HARDWARE MODE ({len(self.motors)} motors on {self.can0_if}/{self.can1_if})")
                    self.hw.query_all_physical()
                    if hasattr(self, 'loop') and self.loop:
                        asyncio.run_coroutine_threadsafe(self.broadcast_notice("success", "Đã tự động kết nối robot thật trên can0/can1!"), self.loop)
                except Exception as e:
                    print(f"[Hotplug] Error switching to real mode: {e}")
            elif self.mode == "real" and not can0_present:
                print("[Hotplug] Physical can0 detached. Falling back to SIMULATION MODE...")
                try:
                    self.hw.stop()
                    new_sim = DamiaoArmSimulator("vcan0")
                    new_sim.start()
                    self.hw = new_sim
                    self.motors = self.hw.motors
                    self.mode = "sim"
                    print("[Hotplug] Fallback to SIMULATION MODE active")
                    if hasattr(self, 'loop') and self.loop:
                        asyncio.run_coroutine_threadsafe(self.broadcast_notice("warning", "Cáp USB đã tháo. Đang hoạt động ở chế độ Mô phỏng (vcan0)"), self.loop)
                except Exception as e:
                    print(f"[Hotplug] Error falling back to sim mode: {e}")

    async def ws_handler(self, websocket):
        """Handle incoming WebSocket client connections and dispatch actions."""
        self.clients.add(websocket)
        try:
            with self.hw.lock:
                motors_data = [m.to_dict() for m in self.motors.values()]
                rx_count = self.hw.frames_rx
                tx_count = self.hw.frames_tx
                now_ts = time.time()
                last_t = self.stream_stats["last_time"]
                stream_info = {
                    "active": (now_ts - last_t < 1.2) if last_t > 0 else False,
                    "packets": self.stream_stats["packets"],
                    "hz": round(self.stream_stats["hz"], 1) if (now_ts - last_t < 1.5) else 0.0,
                    "source": self.stream_stats["source"],
                    "last_ago": round(now_ts - last_t, 2) if last_t > 0 else -1
                }
            init_msg = json.dumps({
                "type": "telemetry",
                "data": {
                    "motors": motors_data,
                    "frames_rx": rx_count,
                    "frames_tx": tx_count,
                    "mode": self.mode,
                    "initial": True,
                    "stream_stats": stream_info
                }
            })
            await websocket.send(init_msg)
        except Exception:
            pass

        try:
            async for msg in websocket:
                try:
                    payload = json.loads(msg)
                    action = payload.get("action")
                    await self.handle_action(action, payload, websocket)
                except Exception as e:
                    print("[WS Action Error]:", e)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.discard(websocket)

    async def handle_action(self, action: str, payload: dict, ws):
        """Process incoming dashboard commands (enable/disable, zero calibration, MIT, Gripper, USB)."""
        if action == "connect_usb":
            print("[Command] Connect USB Robot requested from Web UI")
            threading.Thread(target=self._exec_connect_usb, daemon=True).start()

        elif action == "disconnect_usb":
            print("[Command] Disconnect USB Robot requested from Web UI")
            threading.Thread(target=self._exec_disconnect_usb, daemon=True).start()

        elif action == "export_toggle":
            if self.exporter.active:
                self.exporter.close_session()
            else:
                self.exporter.start_session("manual")

        elif action == "export_new_session":
            self.exporter.start_session("manual")

        elif action == "sync_robot_state":
            print("[Command] Sync state from physical robot requested")
            if self.mode == "real":
                self.hw.query_all_physical()
                time.sleep(0.08)
                with self.hw.lock:
                    for m in self.motors.values():
                        m.q_cmd = m.q
                        m.q_target = m.q
                        m.q_des = m.q
                        m.has_physical_sync = True
                if hasattr(self, 'loop') and self.loop:
                    asyncio.run_coroutine_threadsafe(self.broadcast_notice("success", "Đã đọc và đồng bộ góc khớp thực tế từ Robot!"), self.loop)

        elif action == "set_velocity_limit":
            val = float(payload.get("v_limit", 0.25))
            self.velocity_limit = max(0.02, min(3.0, val))
            print(f"[Speed Profile] Velocity limit set to: {self.velocity_limit:.3f} rad/s ({math.degrees(self.velocity_limit):.1f}°/s)")

        elif action == "enable_all":
            print("[Command] Enable All Motors")
            with self.hw.lock:
                for m in self.motors.values():
                    m.enabled = True
                    m.error_code = 1
                    m.q_cmd = m.q
                    m.q_target = m.q
                    m.q_des = m.q
            if self.mode == "real":
                def _bg_enable():
                    for m in self.motors.values():
                        if m.joint_idx == 8:
                            self.hw.init_gripper_motor(m.id, save_flash=False)
                        else:
                            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                threading.Thread(target=_bg_enable, daemon=True).start()
            else:
                for m in self.hw.motors.values():
                    m.enabled = True
                    m.error_code = 1
                    self.hw._send_can(m.send_id, bytes([0xFF] * 7 + [0xFC]))

        elif action == "disable_all":
            print("[Command] Disarm / Disable All Motors")
            with self.hw.lock:
                for m in self.motors.values():
                    m.enabled = False
                    m.error_code = 0
                    m.q_cmd = m.q
                    m.q_target = m.q
            if self.mode == "real":
                for m in self.motors.values():
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFD]))
            else:
                for m in self.hw.motors.values():
                    m.enabled = False
                    m.error_code = 0
                    self.hw._send_can(m.send_id, bytes([0xFF] * 7 + [0xFD]))

        elif action == "enable_motor":
            motor_id = int(payload.get("id", 1))
            m = self.motors.get(motor_id)
            if m:
                with self.hw.lock:
                    m.enabled = True
                    m.error_code = 1
                    m.q_cmd = m.q
                    m.q_target = m.q
                    m.q_des = m.q
                if self.mode == "real":
                    if m.joint_idx == 8:
                        self.hw.init_gripper_motor(m.id, save_flash=False)
                    else:
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                else:
                    m.enabled = True
                    m.error_code = 1
                    self.hw._send_can(m.send_id, bytes([0xFF] * 7 + [0xFC]))

        elif action == "disable_motor":
            motor_id = int(payload.get("id", 1))
            m = self.motors.get(motor_id)
            if m:
                with self.hw.lock:
                    m.enabled = False
                    m.error_code = 0
                    m.q_cmd = m.q
                    m.q_target = m.q
                if self.mode == "real":
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFD]))
                else:
                    m.enabled = False
                    m.error_code = 0
                    self.hw._send_can(m.send_id, bytes([0xFF] * 7 + [0xFD]))

        elif action == "set_zero_all":
            print("[Command] Set Zero All")
            if self.mode == "real":
                threading.Thread(target=self._exec_set_zero_all, daemon=True).start()
            else:
                for m in self.hw.motors.values():
                    m.q = 0.0
                    m.dq = 0.0
                    m.q_des = 0.0
                    self.hw._send_can(m.send_id, bytes([0xFF] * 7 + [0xFE]))

        elif action == "set_zero_single":
            motor_id = int(payload.get("id", 1))
            m = self.motors.get(motor_id)
            if m:
                print(f"[Command] Set Zero Single Motor {motor_id}")
                if self.mode == "real":
                    threading.Thread(target=self._exec_set_zero_single, args=(m,), daemon=True).start()
                else:
                    m.q = 0.0
                    m.dq = 0.0
                    m.q_des = 0.0

        elif action == "clear_error_all":
            print("[Command] Clear Errors")
            if self.mode == "real":
                for m in self.motors.values():
                    self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFB]))
            else:
                for m in self.hw.motors.values():
                    m.error_code = 0
                    self.hw._send_can(m.send_id, bytes([0xFF] * 7 + [0xFB]))

        elif action == "set_mit":
            motor_id = int(payload.get("id", 1))
            q_raw = float(payload.get("q", 0.0))

            # If Joint 8 received via set_mit, route to set_gripper logic
            if motor_id in [8, 16]:
                m = self.motors.get(motor_id)
                if m:
                    pos_m = q_raw / 1000.0 if (q_raw > 0.043 and q_raw <= 43.0) else q_raw
                    safe_pos = min(0.0415, max(0.0, pos_m))
                    invert = self.gripper_invert.get(m.id, False)
                    stroke_ratio = safe_pos / 0.0415
                    ratio = (1.0 - stroke_ratio) if invert else stroke_ratio
                    rad_target = ratio * 1.15

                    if not m.enabled:
                        m.enabled = True
                        if self.mode == "real":
                            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                    elif m.error_code >= 8:
                        if self.mode == "real":
                            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFB]))
                            time.sleep(0.01)
                            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                        m.error_code = 1

                    m.q_target = rad_target
                    m.q_cmd = rad_target
                    if self.mode == "real":
                        posforce_can_id = m.send_id + 0x300
                        vel_uint = 2500  # 25.0 rad/s
                        i_uint = 2200    # 22% safe current limit
                        posforce_data = struct.pack("<fHH", float(rad_target), vel_uint, i_uint)
                        self.hw.send_frame(m.can_if, posforce_can_id, posforce_data)
                        refresh_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                        self.hw.send_frame(m.can_if, 0x7FF, refresh_data)
                return

            kp = float(payload.get("kp", 18.0))
            kd = float(payload.get("kd", 2.0))
            tau = float(payload.get("tau", 0.0))

            motor = self.motors.get(motor_id)
            if motor:
                lim = JOINT_LIMITS.get(motor_id, (-12.5, 12.5))
                q = max(lim[0], min(lim[1], q_raw))
                if not motor.enabled:
                    motor.enabled = True
                    motor.q_cmd = motor.q
                    if self.mode == "real":
                        self.hw.send_frame(motor.can_if, motor.send_id, bytes([0xFF] * 7 + [0xFC]))
                motor.q_target = q
                motor.kp = kp
                motor.kd = kd
                motor.tau_ff = tau

        elif action == "set_gripper":
            pos_raw = float(payload.get("pos", 0.0))
            if pos_raw > 0.043 and pos_raw <= 43.0:
                pos_m = pos_raw / 1000.0
            else:
                pos_m = pos_raw
            pos = max(0.0, min(0.043, pos_m))
            target_arm = payload.get("arm", "both")
            target_id = payload.get("id")

            target_motors = []
            if target_id is not None:
                m = self.motors.get(int(target_id))
                if m:
                    target_motors.append(m)
            elif target_arm == "left":
                m = self.motors.get(8)
                if m:
                    target_motors.append(m)
            elif target_arm == "right":
                m = self.motors.get(16)
                if m:
                    target_motors.append(m)
            else:
                for gid in [8, 16]:
                    m = self.motors.get(gid)
                    if m:
                        target_motors.append(m)

            for m in target_motors:
                safe_pos = min(0.0415, max(0.0, pos))
                invert = self.gripper_invert.get(m.id, False)
                stroke_ratio = safe_pos / 0.0415
                ratio = (1.0 - stroke_ratio) if invert else stroke_ratio
                rad_target = ratio * 1.15

                if not m.enabled:
                    m.enabled = True
                    if self.mode == "real":
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                elif m.error_code >= 8:
                    if self.mode == "real":
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFB]))
                        time.sleep(0.01)
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                    m.error_code = 1

                m.q_target = rad_target
                m.q_cmd = rad_target

                if self.mode == "real":
                    posforce_can_id = m.send_id + 0x300
                    vel_uint = 2500  # 25.0 rad/s
                    i_uint = 2200    # 22% safe current limit
                    posforce_data = struct.pack("<fHH", float(rad_target), vel_uint, i_uint)
                    self.hw.send_frame(m.can_if, posforce_can_id, posforce_data)
                    refresh_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                    self.hw.send_frame(m.can_if, 0x7FF, refresh_data)

                can_name = getattr(m, 'can_if', 'vcan0')
                print(f"[Gripper] Motor {m.id} ({m.name}) on {can_name} target -> stroke {pos*1000:.1f} mm ({rad_target:.3f} rad)")

        elif action == "toggle_gripper_invert":
            target_id = int(payload.get("id", 8))
            self.gripper_invert[target_id] = not self.gripper_invert.get(target_id, True)
            m = self.motors.get(target_id)
            if m:
                m.invert = self.gripper_invert[target_id]
            print(f"[Gripper] Motor {target_id} invert set to: {self.gripper_invert[target_id]}")

        elif action == "run_cli":
            cmd = payload.get("cmd", "")
            target_iface = self.can0_if if self.mode == "real" else "vcan0"
            threading.Thread(target=self._exec_cli, args=(cmd, target_iface), daemon=True).start()

        elif action == "set_joint_state":
            self.apply_joint_states(payload, source="WebSocket")

    def _udp_receiver_loop(self, port: int = UDP_STREAM_PORT):
        """High-performance, zero-latency UDP receiver for real-time external streams (60-200 Hz)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("0.0.0.0", port))
            print(f"[Stream Receiver] High-speed UDP Joint State stream listening on 0.0.0.0:{port}")
            while self.running:
                data, addr = sock.recvfrom(8192)
                try:
                    payload = json.loads(data.decode('utf-8'))
                    self.apply_joint_states(payload, source=f"UDP ({addr[0]}:{addr[1]})")
                except Exception:
                    pass
        except Exception as e:
            print(f"[Stream Receiver] Warning: UDP stream error on port {port}: {e}")
        finally:
            sock.close()

    def apply_joint_states(self, payload: dict, source: str = "External"):
        """
        Apply target joint states from external sources (ROS 2 / AI / Teleop).
        Supports flexible input formats:
        - {"joints": {"openarm_left_joint1": 0.5, ...}}
        - {"names": ["openarm_left_joint1", ...], "positions": [0.5, ...]}
        - {"left": [q1..q7], "right": [q9..q15], "left_gripper": pos, "right_gripper": pos}
        - {"positions": [16 values in order 1..16]}
        """
        now = time.time()
        prev = self.stream_stats["last_time"]
        dt = now - prev if prev > 0 else 0
        if 0 < dt < 2.0:
            instant_hz = 1.0 / dt
            self.stream_stats["hz"] = self.stream_stats["hz"] * 0.85 + instant_hz * 0.15
        elif dt >= 2.0:
            self.stream_stats["hz"] = 1.0
        self.stream_stats["last_time"] = now
        self.stream_stats["packets"] += 1
        self.stream_stats["source"] = source

        joint_map = {}

        # Format 1: names + positions lists (standard sensor_msgs/JointState)
        if "names" in payload and "positions" in payload:
            for name, pos in zip(payload["names"], payload["positions"]):
                mid = JOINT_NAME_TO_ID.get(str(name).lower())
                if mid:
                    joint_map[mid] = float(pos)

        # Format 2: direct "joints" dictionary
        elif "joints" in payload and isinstance(payload["joints"], dict):
            for k, v in payload["joints"].items():
                if str(k).isdigit():
                    mid = int(k)
                else:
                    mid = JOINT_NAME_TO_ID.get(str(k).lower())
                if mid and (1 <= mid <= 16):
                    joint_map[mid] = float(v)

        # Format 3: "left" and "right" lists
        elif "left" in payload or "right" in payload:
            if "left" in payload and isinstance(payload["left"], list):
                for idx, val in enumerate(payload["left"][:7]):
                    joint_map[idx + 1] = float(val)
            if "left_gripper" in payload:
                joint_map[8] = float(payload["left_gripper"])

            if "right" in payload and isinstance(payload["right"], list):
                for idx, val in enumerate(payload["right"][:7]):
                    joint_map[idx + 9] = float(val)
            if "right_gripper" in payload:
                joint_map[16] = float(payload["right_gripper"])

        # Format 4: flat list of 16 positions
        elif "positions" in payload and isinstance(payload["positions"], list):
            for idx, val in enumerate(payload["positions"][:16]):
                joint_map[idx + 1] = float(val)

        # Format 5: direct top-level joint mapping (e.g. {"openarm_left_joint1": 0.5})
        elif isinstance(payload, dict):
            for k, v in payload.items():
                if str(k).isdigit():
                    mid = int(k)
                else:
                    mid = JOINT_NAME_TO_ID.get(str(k).lower())
                if mid and (1 <= mid <= 16):
                    try:
                        joint_map[mid] = float(v)
                    except (ValueError, TypeError):
                        pass

        # Apply to motors
        for motor_id, val in joint_map.items():
            m = self.motors.get(motor_id)
            if not m:
                continue

            if motor_id in [8, 16]:
                # Gripper (Joint 8 / Joint 16)
                pos_m = val / 1000.0 if (val > 0.043 and val <= 43.0) else val
                safe_pos = min(0.0415, max(0.0, pos_m))
                invert = self.gripper_invert.get(motor_id, False)
                stroke_ratio = safe_pos / 0.0415
                ratio = (1.0 - stroke_ratio) if invert else stroke_ratio
                rad_target = ratio * 1.15

                if not m.enabled:
                    m.enabled = True
                    if self.mode == "real":
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                elif m.error_code >= 8:
                    if self.mode == "real":
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFB]))
                        time.sleep(0.01)
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                    m.error_code = 1

                m.q_target = rad_target
                m.q_cmd = rad_target

                if self.mode == "real":
                    posforce_can_id = m.send_id + 0x300
                    vel_uint = 2500
                    i_uint = 2200
                    posforce_data = struct.pack("<fHH", float(rad_target), vel_uint, i_uint)
                    self.hw.send_frame(m.can_if, posforce_can_id, posforce_data)
                    refresh_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                    self.hw.send_frame(m.can_if, 0x7FF, refresh_data)
            else:
                # Arm Joints (1..7, 9..15)
                lim = JOINT_LIMITS.get(motor_id, (-3.1415, 3.1415))
                q = max(lim[0], min(lim[1], val))
                if not m.enabled:
                    m.enabled = True
                    m.q_cmd = m.q
                    if self.mode == "real":
                        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFC]))
                m.q_target = q

    def _trajectory_loop(self):
        """400 Hz trajectory generator & control loop that smoothly moves motors at limited velocity."""
        dt = 1.0 / CONTROL_FREQ  # 0.0025s (2.5 ms)
        next_tick = time.perf_counter()

        while self.running:
            try:
                for motor_id, m in self.motors.items():
                    if not m.enabled:
                        m.q_cmd = m.q
                        m.q_target = m.q
                        m.q_des = m.q
                        continue

                    # Velocity-limited step towards target
                    diff = m.q_target - m.q_cmd
                    v_lim = 2.5 if m.joint_idx == 8 else self.velocity_limit
                    max_step = v_lim * dt

                    if abs(diff) <= max_step:
                        m.q_cmd = m.q_target
                    else:
                        m.q_cmd += math.copysign(max_step, diff)

                    m.q_des = m.q_cmd

                    # If in real mode and motor is enabled, send CAN command
                    if self.mode == "real":
                        if m.joint_idx == 8:
                            # End-Effector parallel gripper (DM4310 in POS_FORCE)
                            now = time.time()
                            if now - getattr(m, '_last_posforce_tx', 0) > 0.05:
                                m._last_posforce_tx = now
                                posforce_can_id = m.send_id + 0x300
                                vel_uint = 2500  # 25.0 rad/s
                                i_uint = 2200    # 0.22 pu safe torque limit
                                posforce_data = struct.pack("<fHH", float(m.q_target), vel_uint, i_uint)
                                self.hw.send_frame(m.can_if, posforce_can_id, posforce_data)
                                refresh_data = bytes([m.send_id & 0xFF, (m.send_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
                                self.hw.send_frame(m.can_if, 0x7FF, refresh_data)
                        else:
                            # 7-DOF Arm motors (MIT Mode)
                            q_uint = double_to_uint(m.q_cmd, -m.pMax, m.pMax, 16)
                            dq_uint = double_to_uint(0.0, -m.vMax, m.vMax, 12)
                            kp_uint = double_to_uint(m.kp, 0.0, 500.0, 12)
                            kd_uint = double_to_uint(m.kd, 0.0, 5.0, 12)
                            tau_uint = double_to_uint(0.0, -m.tMax, m.tMax, 12)

                            d0 = (q_uint >> 8) & 0xFF
                            d1 = q_uint & 0xFF
                            d2 = (dq_uint >> 4) & 0xFF
                            d3 = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
                            d4 = kp_uint & 0xFF
                            d5 = (kd_uint >> 4) & 0xFF
                            d6 = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
                            d7 = tau_uint & 0xFF
                            mit_data = bytes([d0, d1, d2, d3, d4, d5, d6, d7])
                            self.hw.send_frame(m.can_if, m.send_id, mit_data)
                    else:
                        m.q = m.q_cmd

            except Exception:
                pass

            next_tick += dt
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0.0005:
                time.sleep(sleep_time)
            elif sleep_time < -0.05:
                next_tick = time.perf_counter()

    def _exec_set_zero_all(self):
        """Execute true DaMiao Zero Calibration sequence: Disable -> Set Zero -> Disable, then reset state."""
        print("[Hardware] Executing mechanical zero calibration sequence on all motors...")
        for m in self.motors.values():
            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFD]))
        time.sleep(0.1)

        for m in self.motors.values():
            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFE]))
        time.sleep(0.12)

        for m in self.motors.values():
            self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFD]))
        time.sleep(0.08)

        with self.hw.lock:
            for m in self.motors.values():
                m.q = 0.0
                m.q_cmd = 0.0
                m.q_target = 0.0
                m.dq = 0.0
                m.q_des = 0.0
                m.tau = 0.0
        print("[Hardware] Zero calibration complete. All motor positions reset to 0.0 rad.")

    def _exec_set_zero_single(self, m):
        """Execute true DaMiao Zero Calibration sequence for a single motor."""
        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFD]))
        time.sleep(0.1)
        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFE]))
        time.sleep(0.12)
        self.hw.send_frame(m.can_if, m.send_id, bytes([0xFF] * 7 + [0xFD]))
        time.sleep(0.05)
        with self.hw.lock:
            m.q = 0.0
            m.q_cmd = 0.0
            m.q_target = 0.0
            m.dq = 0.0
            m.q_des = 0.0
            m.tau = 0.0

    def _exec_cli(self, cmd: str, iface: str):
        """Run openarm-can-cli diagnostics and pipe output to WebSocket."""
        full_cmd = ["openarm-can-cli", "-i", iface]
        if cmd == "discover":
            full_cmd += ["discover"]
        elif cmd == "show_param":
            full_cmd += ["show_param", "--id", "1"]
        elif cmd == "monitor":
            full_cmd += ["monitor", "--id", "1,2,3,4,5,6,7,8"]
        elif cmd == "diagnose":
            full_cmd += ["diagnose"]
        else:
            return

        try:
            res = subprocess.run(full_cmd, capture_output=True, text=True, timeout=10)
            out = res.stdout if res.stdout else res.stderr
        except Exception as e:
            out = f"Execution error: {e}"

        asyncio.run_coroutine_threadsafe(self.broadcast_cli_output(out), self.loop)

    async def broadcast_cli_output(self, text: str):
        msg = json.dumps({"type": "cli_output", "data": text})
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except Exception:
                pass

    async def broadcast_telemetry_loop(self):
        """Broadcast state packets to connected WebSocket clients at ~40 Hz."""
        traffic_tick = 0
        while self.running:
            if self.clients:
                with self.hw.lock:
                    motors_data = [m.to_dict() for m in self.motors.values()]
                    rx_count = self.hw.frames_rx
                    tx_count = self.hw.frames_tx

                    now_ts = time.time()
                    last_t = self.stream_stats["last_time"]
                    stream_info = {
                        "active": (now_ts - last_t < 1.2) if last_t > 0 else False,
                        "packets": self.stream_stats["packets"],
                        "hz": round(self.stream_stats["hz"], 1) if (now_ts - last_t < 1.5) else 0.0,
                        "source": self.stream_stats["source"],
                        "last_ago": round(now_ts - last_t, 2) if last_t > 0 else -1
                    }

                telem_msg = json.dumps({
                    "type": "telemetry",
                    "data": {
                        "motors": motors_data,
                        "frames_rx": rx_count,
                        "frames_tx": tx_count,
                        "mode": self.mode,
                        "stream_stats": stream_info,
                        "export_stats": self.exporter.get_stats()
                    }
                })

                traffic_tick += 1
                traffic_msg = None
                if traffic_tick % 2 == 0:
                    with self.hw.lock:
                        recent_traffic = list(self.hw.traffic_log)
                    traffic_msg = json.dumps({
                        "type": "traffic",
                        "data": recent_traffic
                    })

                dead_clients = set()
                for ws in list(self.clients):
                    try:
                        await ws.send(telem_msg)
                        if traffic_msg:
                            await ws.send(traffic_msg)
                    except Exception:
                        dead_clients.add(ws)

                self.clients.difference_update(dead_clients)

            await asyncio.sleep(1.0 / TELEMETRY_FREQ)

    async def run_ws_server(self):
        """Launch asyncio WebSocket server on port 8889."""
        self.loop = asyncio.get_running_loop()
        server = await websockets.serve(self.ws_handler, "0.0.0.0", WS_PORT)
        print(f"[Dashboard] WebSocket Server running on ws://localhost:{WS_PORT} (Mode: {self.mode.upper()})")
        await self.broadcast_telemetry_loop()
