#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc. / OpenArm Control & Simulation
"""
HTTP static file server and REST API handler for the OpenArm web dashboard.
"""

import json
import os
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

try:
    from config import WEB_DIR
except ImportError:
    from .config import WEB_DIR


class CustomHTTPHandler(SimpleHTTPRequestHandler):
    """HTTP handler serving web assets and responding to REST endpoints."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def do_GET(self):
        if self.path == "/api/export/status":
            if hasattr(self.server, 'app') and self.server.app:
                stats = self.server.app.exporter.get_stats()
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(stats).encode('utf-8'))
                return
        elif self.path == "/api/export/download":
            if hasattr(self.server, 'app') and self.server.app:
                exporter = self.server.app.exporter
                file_path = exporter.file_path or exporter.get_latest_file()
                if file_path and os.path.exists(file_path):
                    if exporter.file:
                        try:
                            exporter.file.flush()
                        except Exception:
                            pass
                    with open(file_path, "rb") as f:
                        data = f.read()
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/csv')
                    self.send_header('Content-Disposition', f'attachment; filename="{os.path.basename(file_path)}"')
                    self.send_header('Content-Length', str(len(data)))
                    self.end_headers()
                    self.wfile.write(data)
                    return
                else:
                    self.send_response(404)
                    self.send_header('Content-Type', 'application/json')
                    self.end_headers()
                    self.wfile.write(b'{"error": "No export file found"}')
                    return
        elif self.path == "/api/export/new_session":
            if hasattr(self.server, 'app') and self.server.app:
                self.server.app.exporter.start_session("manual")
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(self.server.app.exporter.get_stats()).encode('utf-8'))
                return

        super().do_GET()

    def do_POST(self):
        if self.path == "/api/joint_state":
            length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(length)
            try:
                data = json.loads(body.decode('utf-8'))
                if hasattr(self.server, 'app') and self.server.app:
                    self.server.app.apply_joint_states(data, source="REST HTTP")
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status": "ok"}')
            except Exception as e:
                self.send_response(400)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode('utf-8'))
            return
        elif self.path == "/api/export/toggle":
            if hasattr(self.server, 'app') and self.server.app:
                exporter = self.server.app.exporter
                if exporter.active:
                    exporter.close_session()
                else:
                    exporter.start_session("manual")
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(exporter.get_stats()).encode('utf-8'))
                return
        elif self.path == "/api/export/new_session":
            if hasattr(self.server, 'app') and self.server.app:
                self.server.app.exporter.start_session("manual")
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(self.server.app.exporter.get_stats()).encode('utf-8'))
                return

        super().do_POST()

    def end_headers(self):
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, format, *args):
        pass
