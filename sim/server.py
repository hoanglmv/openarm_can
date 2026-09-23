#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Simulation
"""
Unified Server for OpenArm CAN Simulation & Web Dashboard:
- Runs Virtual Damiao Arm CAN-FD Simulator on vcan0
- Serves HTTP static files (HTML/CSS/JS) on port 8888
- Serves WebSocket real-time telemetry and control on port 8889
- Executes openarm-can-cli subcommands on demand
"""

import os
import sys
import time
import json
import socket
import struct
import asyncio
import threading
import subprocess
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

import websockets

from motor_simulator import (
    DamiaoArmSimulator,
    double_to_uint,
    CANFD_FRAME_FMT,
    CAN_RAW_FD_FRAMES,
    SOL_CAN_RAW
)

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
HTTP_PORT = 8888
WS_PORT = 8889

class CustomHTTPHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def log_message(self, format, *args):
        # Suppress noisy HTTP request logging
        pass


class OpenArmDashboardServer:
    def __init__(self, interface: str = "vcan0"):
        self.interface = interface
        self.sim = DamiaoArmSimulator(interface)
        self.clients = set()
        self.running = True

        # Master CAN sender socket
        self.master_sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.master_sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
        except Exception:
            pass
        self.master_sock.bind((self.interface,))

    def start(self):
        # 1. Start Virtual Damiao CAN-FD Simulator
        self.sim.start()

        # 2. Start HTTP server thread
        self.httpd = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), CustomHTTPHandler)
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        print(f"[Dashboard] HTTP Server running on http://localhost:{HTTP_PORT}")

        # 3. Start WebSocket & Telemetry Broadcaster
        asyncio.run(self.run_ws_server())

    def send_can_master(self, can_id: int, data: bytes):
        """Send CAN-FD frame as host controller"""
        pad = 64 - len(data)
        frame = struct.pack(CANFD_FRAME_FMT, can_id, len(data), 0, 0, data + (b'\x00' * pad))
        try:
            self.master_sock.send(frame)
        except Exception as e:
            print("[Master Send Error]:", e)

    async def ws_handler(self, websocket):
        self.clients.add(websocket)
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
        if action == "enable_all":
            for motor in self.sim.motors.values():
                self.send_can_master(motor.send_id, bytes([0xFF]*7 + [0xFC]))

        elif action == "disable_all":
            for motor in self.sim.motors.values():
                self.send_can_master(motor.send_id, bytes([0xFF]*7 + [0xFD]))

        elif action == "set_zero_all":
            for motor in self.sim.motors.values():
                self.send_can_master(motor.send_id, bytes([0xFF]*7 + [0xFE]))

        elif action == "clear_error_all":
            for motor in self.sim.motors.values():
                self.send_can_master(motor.send_id, bytes([0xFF]*7 + [0xFB]))

        elif action == "set_mit":
            motor_id = int(payload.get("id", 1))
            q = float(payload.get("q", 0.0))
            kp = float(payload.get("kp", 30.0))
            kd = float(payload.get("kd", 1.2))
            tau = float(payload.get("tau", 0.0))

            motor = self.sim.motors.get(motor_id)
            if motor:
                # Pack MIT frame exactly as CanPacketEncoder does
                q_uint = double_to_uint(q, -motor.pMax, motor.pMax, 16)
                dq_uint = double_to_uint(0.0, -motor.vMax, motor.vMax, 12)
                kp_uint = double_to_uint(kp, 0.0, 500.0, 12)
                kd_uint = double_to_uint(kd, 0.0, 5.0, 12)
                tau_uint = double_to_uint(tau, -motor.tMax, motor.tMax, 12)

                d0 = (q_uint >> 8) & 0xFF
                d1 = q_uint & 0xFF
                d2 = (dq_uint >> 4) & 0xFF
                d3 = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
                d4 = kp_uint & 0xFF
                d5 = (kd_uint >> 4) & 0xFF
                d6 = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
                d7 = tau_uint & 0xFF
                mit_data = bytes([d0, d1, d2, d3, d4, d5, d6, d7])

                self.send_can_master(motor.send_id, mit_data)

        elif action == "set_gripper":
            pos = float(payload.get("pos", 0.0))
            gripper = self.sim.motors.get(8)
            if gripper:
                # Send pos command via MIT mode
                q_uint = double_to_uint(pos, -gripper.pMax, gripper.pMax, 16)
                dq_uint = double_to_uint(0.0, -gripper.vMax, gripper.vMax, 12)
                kp_uint = double_to_uint(40.0, 0.0, 500.0, 12)
                kd_uint = double_to_uint(1.5, 0.0, 5.0, 12)
                tau_uint = double_to_uint(0.0, -gripper.tMax, gripper.tMax, 12)

                d0 = (q_uint >> 8) & 0xFF
                d1 = q_uint & 0xFF
                d2 = (dq_uint >> 4) & 0xFF
                d3 = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F)
                d4 = kp_uint & 0xFF
                d5 = (kd_uint >> 4) & 0xFF
                d6 = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F)
                d7 = tau_uint & 0xFF
                self.send_can_master(gripper.send_id, bytes([d0, d1, d2, d3, d4, d5, d6, d7]))

        elif action == "run_cli":
            cmd = payload.get("cmd", "")
            root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            cli_bin = os.path.join(root_dir, "build", "openarm-can-cli")

            args = [cli_bin, "-i", self.interface]
            if cmd == "discover":
                args += ["discover", "-m", "8"]
            elif cmd == "show_param":
                args += ["show_param", "--id", "1"]
            elif cmd == "monitor":
                args += ["monitor", "--duration", "2000"]
            elif cmd == "diagnose":
                args += ["diagnose", "--duration", "2000"]
            else:
                args += [cmd]

            def execute_cli():
                try:
                    res = subprocess.run(args, capture_output=True, text=True, timeout=12)
                    output = res.stdout
                    if res.stderr:
                        output += "\n" + res.stderr
                    if res.returncode != 0 and not output:
                        output += f"\n[Process exited with code {res.returncode}]"
                except Exception as ex:
                    output = f"Error executing CLI: {str(ex)}"
                finally:
                    # Virtual CAN interfaces cannot have hardware bitrate changed, ensure it stays UP
                    subprocess.run(["sudo", "ip", "link", "set", "up", self.interface], capture_output=True)

                if ws in self.clients:
                    asyncio.run_coroutine_threadsafe(
                        ws.send(json.dumps({"type": "cli_output", "data": output})),
                        self.loop
                    )

            threading.Thread(target=execute_cli, daemon=True).start()

    async def broadcast_telemetry_loop(self):
        """Broadcast 40 Hz telemetry and traffic packets to all WebSockets"""
        traffic_tick = 0
        while self.running:
            if self.clients:
                with self.sim.lock:
                    motors_data = [m.to_dict() for m in self.sim.motors.values()]
                    rx_count = self.sim.frames_rx
                    tx_count = self.sim.frames_tx

                telem_msg = json.dumps({
                    "type": "telemetry",
                    "data": {
                        "motors": motors_data,
                        "frames_rx": rx_count,
                        "frames_tx": tx_count
                    }
                })

                traffic_tick += 1
                traffic_msg = None
                if traffic_tick % 2 == 0:
                    with self.sim.lock:
                        recent_traffic = list(self.sim.traffic_log)
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

            await asyncio.sleep(0.025) # 40 Hz

    async def run_ws_server(self):
        self.loop = asyncio.get_running_loop()
        server = await websockets.serve(self.ws_handler, "0.0.0.0", WS_PORT)
        print(f"[Dashboard] WebSocket Server running on ws://localhost:{WS_PORT}")
        await self.broadcast_telemetry_loop()


if __name__ == "__main__":
    server = OpenArmDashboardServer("vcan0")
    server.start()
