"""Minimal MJPEG HTTP server, stdlib-only (http.server), used by
perception_node.py to expose a live preview stream to the GCS without any
new pip dependency.

Deliberately dumb: this module only serves whatever JPEG bytes it's handed
via update_frame() -- encoding (cv2.imencode) is the caller's job, not this
module's.
"""
from __future__ import annotations

import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

logger = logging.getLogger(__name__)

_BOUNDARY = "frame"
_STREAM_POLL_INTERVAL_S = 0.05


class _FrameHolder:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None

    def set(self, jpeg_bytes: bytes) -> None:
        with self._lock:
            self._jpeg = jpeg_bytes

    def get(self) -> bytes | None:
        with self._lock:
            return self._jpeg


def _make_handler(holder: _FrameHolder):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # noqa: A003 -- silence stdlib access logging
            pass

        def do_GET(self) -> None:  # noqa: N802 -- stdlib API name
            if self.path == "/healthz":
                self._handle_healthz()
            elif self.path == "/snapshot.jpg":
                self._handle_snapshot()
            elif self.path == "/stream.mjpg":
                self._handle_stream()
            else:
                self.send_error(404)

        def _handle_healthz(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(b"OK")

        def _handle_snapshot(self) -> None:
            jpeg = holder.get()
            if jpeg is None:
                self.send_error(503, "no frame available yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)

        def _handle_stream(self) -> None:
            self.send_response(200)
            self.send_header(
                "Content-Type", f"multipart/x-mixed-replace; boundary={_BOUNDARY}"
            )
            self.end_headers()
            try:
                while True:
                    jpeg = holder.get()
                    if jpeg is not None:
                        self.wfile.write(f"--{_BOUNDARY}\r\n".encode())
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                    time.sleep(_STREAM_POLL_INTERVAL_S)
            except (BrokenPipeError, ConnectionResetError):
                pass  # client disconnected -- not a server error

    return Handler


class MJPEGStreamServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 8090) -> None:
        self._host = host
        self._port = port
        self._holder = _FrameHolder()
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    def update_frame(self, jpeg_bytes: bytes) -> None:
        self._holder.set(jpeg_bytes)

    def start(self) -> None:
        if self._httpd is not None:
            return
        self._httpd = ThreadingHTTPServer((self._host, self._port), _make_handler(self._holder))
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        logger.info("MJPEGStreamServer listening on %s:%d", self._host, self._port)

    def stop(self) -> None:
        if self._httpd is None:
            return
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        self._httpd = None
        self._thread = None
