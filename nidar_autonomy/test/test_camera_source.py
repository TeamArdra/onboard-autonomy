"""Pure-logic / mocked-cv2 tests for perception/camera_source.py.

SyntheticCameraSource needs no ROS/hardware; V4L2CameraSource is tested with
a monkeypatched cv2.VideoCapture so no real device is required either. No
ROS import at module scope -- ROSImageCameraSource is covered separately
(it lazily imports rclpy inside __init__, so is untestable without a real
ROS environment and is out of scope for this file).
"""
import numpy as np
import pytest

from nidar_autonomy.perception.camera_source import SyntheticCameraSource, V4L2CameraSource


class TestSyntheticCameraSource:
    def test_not_connected_before_open(self):
        cam = SyntheticCameraSource(width=64, height=48, seed=1)
        assert cam.is_connected is False
        assert cam.read() is None

    def test_connected_after_open_and_produces_correctly_shaped_frames(self):
        cam = SyntheticCameraSource(width=64, height=48, seed=1)
        cam.open()
        assert cam.is_connected is True
        frame = cam.read()
        assert frame is not None
        assert frame.width == 64
        assert frame.height == 48
        assert frame.image.shape == (48, 64, 3)
        assert frame.image.dtype == np.uint8

    def test_frame_id_increments(self):
        cam = SyntheticCameraSource(width=32, height=24, seed=1)
        cam.open()
        f0 = cam.read()
        f1 = cam.read()
        f2 = cam.read()
        assert [f0.frame_id, f1.frame_id, f2.frame_id] == [0, 1, 2]

    def test_deterministic_given_same_seed(self):
        cam_a = SyntheticCameraSource(width=32, height=24, seed=42)
        cam_a.open()
        cam_b = SyntheticCameraSource(width=32, height=24, seed=42)
        cam_b.open()
        frame_a = cam_a.read()
        frame_b = cam_b.read()
        assert np.array_equal(frame_a.image, frame_b.image)

    def test_close_disconnects(self):
        cam = SyntheticCameraSource(width=32, height=24)
        cam.open()
        cam.close()
        assert cam.is_connected is False
        assert cam.read() is None


class _FakeCapOpens:
    def __init__(self, *_args, **_kwargs):
        self._opened = True
        self.set_calls = []

    def set(self, prop, value):
        self.set_calls.append((prop, value))

    def isOpened(self):
        return self._opened

    def read(self):
        return True, np.zeros((48, 64, 3), dtype=np.uint8)

    def release(self):
        self._opened = False


class _FakeCapFailsToOpen:
    def __init__(self, *_args, **_kwargs):
        pass

    def set(self, prop, value):
        pass

    def isOpened(self):
        return False

    def read(self):
        return False, None

    def release(self):
        pass


class TestV4L2CameraSource:
    def test_opens_and_reads_fine(self, monkeypatch):
        import cv2

        monkeypatch.setattr(cv2, "VideoCapture", _FakeCapOpens)
        cam = V4L2CameraSource(device="/dev/video0", width=64, height=48)
        cam.open()
        assert cam.is_connected is True
        frame = cam.read()
        assert frame is not None
        assert frame.width == 64
        assert frame.height == 48
        cam.close()
        assert cam.is_connected is False

    def test_device_does_not_exist_never_raises(self, monkeypatch):
        import cv2

        monkeypatch.setattr(cv2, "VideoCapture", _FakeCapFailsToOpen)
        cam = V4L2CameraSource(device="/dev/video99", width=64, height=48)
        cam.open()  # must not raise
        assert cam.is_connected is False
        assert cam.read() is None  # must not raise

    def test_read_before_open_returns_none(self):
        cam = V4L2CameraSource(device="/dev/video0")
        assert cam.read() is None
        assert cam.is_connected is False

    def test_close_before_open_does_not_raise(self):
        cam = V4L2CameraSource(device="/dev/video0")
        cam.close()  # must not raise
