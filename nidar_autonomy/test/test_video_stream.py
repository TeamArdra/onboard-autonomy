"""Tests for perception/video_stream.py -- starts a real MJPEGStreamServer
on an ephemeral port and hits it with real HTTP requests (stdlib
urllib.request, no new test dependency)."""
import http.client
import socket
import urllib.error
import urllib.request

import pytest

from nidar_autonomy.perception.video_stream import MJPEGStreamServer


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def server():
    port = _free_port()
    srv = MJPEGStreamServer(host="127.0.0.1", port=port)
    srv.start()
    yield srv, port
    srv.stop()


class TestMJPEGStreamServer:
    def test_healthz_returns_200(self, server):
        srv, port = server
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/healthz", timeout=5) as resp:
            assert resp.status == 200
            assert resp.read() == b"OK"

    def test_snapshot_returns_503_with_no_frame_yet(self, server):
        srv, port = server
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/snapshot.jpg", timeout=5)
            assert False, "expected an HTTPError"
        except urllib.error.HTTPError as exc:
            assert exc.code == 503

    def test_snapshot_serves_the_latest_frame(self, server):
        srv, port = server
        jpeg_bytes = b"\xff\xd8\xff\xe0fake-jpeg-bytes"
        srv.update_frame(jpeg_bytes)
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/snapshot.jpg", timeout=5) as resp:
            assert resp.status == 200
            assert resp.headers["Content-Type"] == "image/jpeg"
            assert resp.read() == jpeg_bytes

    def test_unknown_path_returns_404(self, server):
        srv, port = server
        try:
            urllib.request.urlopen(f"http://127.0.0.1:{port}/nope", timeout=5)
            assert False, "expected an HTTPError"
        except urllib.error.HTTPError as exc:
            assert exc.code == 404

    def test_stream_endpoint_serves_multipart_headers(self, server):
        srv, port = server
        srv.update_frame(b"\xff\xd8\xff\xe0fake-jpeg-bytes")
        conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        try:
            conn.request("GET", "/stream.mjpg")
            resp = conn.getresponse()
            assert resp.status == 200
            assert "multipart/x-mixed-replace" in resp.getheader("Content-Type")
            chunk = resp.read(64)
            assert b"--frame" in chunk
        finally:
            conn.close()

    def test_stop_releases_the_port(self, server):
        srv, port = server
        srv.stop()
        # A second server must be able to bind the same port after stop().
        srv2 = MJPEGStreamServer(host="127.0.0.1", port=port)
        srv2.start()
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{port}/healthz", timeout=5) as resp:
                assert resp.status == 200
        finally:
            srv2.stop()

    def test_start_is_idempotent(self, server):
        srv, port = server
        srv.start()  # must not raise / must not rebind
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/healthz", timeout=5) as resp:
            assert resp.status == 200
