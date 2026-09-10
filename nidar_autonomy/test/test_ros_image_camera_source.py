"""Tests for perception/camera_source.py's ROSImageCameraSource -- the one
CameraSource implementation test_camera_source.py explicitly leaves out
(it lazily imports rclpy inside __init__, so it needs a real ROS
environment). Gated with pytest.importorskip("rclpy"), same pattern as
test_perception_node.py / test_detector_config.py.

Messages are fed directly through camera._on_image(msg) (mirrors
gcs/backend's test_ros_client.py "feed the handler directly" pattern) --
no real publisher/spin loop is needed to exercise the decode logic.
"""
import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

from nidar_autonomy.perception.camera_source import ROSImageCameraSource  # noqa: E402


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_image_msg(width: int, height: int, encoding: str, fill_value: int = 200) -> Image:
    channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}[encoding]
    msg = Image()
    msg.width = width
    msg.height = height
    msg.encoding = encoding
    msg.step = width * channels
    msg.data = bytes([fill_value]) * (msg.step * height)
    return msg


class TestNotConnectedBeforeAnyMessage:
    def test_is_connected_false_and_read_none_before_any_image(self):
        node = rclpy.create_node("test_ros_image_camera_not_connected")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            assert camera.is_connected is False
            assert camera.read() is None
        finally:
            node.destroy_node()


class TestFrameDecoding:
    def test_bgr8_message_decodes_to_correctly_shaped_frame(self):
        node = rclpy.create_node("test_ros_image_camera_bgr8")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            camera._on_image(_make_image_msg(64, 48, "bgr8"))

            assert camera.is_connected is True
            frame = camera.read()
            assert frame is not None
            assert frame.width == 64
            assert frame.height == 48
            assert frame.image.shape == (48, 64, 3)
            assert frame.image.dtype.name == "uint8"
        finally:
            node.destroy_node()

    def test_rgb8_message_channel_order_is_reversed_to_bgr(self):
        node = rclpy.create_node("test_ros_image_camera_rgb8")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            msg = Image()
            msg.width = 2
            msg.height = 1
            msg.encoding = "rgb8"
            msg.step = 2 * 3
            # pixel 0 red (255,0,0), pixel 1 green (0,255,0), in RGB order on the wire
            msg.data = bytes([255, 0, 0, 0, 255, 0])
            camera._on_image(msg)

            frame = camera.read()
            assert list(frame.image[0, 0]) == [0, 0, 255]  # decoded BGR: red -> (0,0,255)
            assert list(frame.image[0, 1]) == [0, 255, 0]  # green is channel-symmetric either way
        finally:
            node.destroy_node()

    def test_mono8_message_is_replicated_across_three_channels(self):
        node = rclpy.create_node("test_ros_image_camera_mono8")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            camera._on_image(_make_image_msg(4, 3, "mono8", fill_value=77))

            frame = camera.read()
            assert frame.image.shape == (3, 4, 3)
            assert (frame.image == 77).all()
        finally:
            node.destroy_node()

    def test_unknown_encoding_leaves_camera_connected_but_read_returns_none(self):
        """A message DID arrive (is_connected reflects that), but it can't
        be decoded into a Frame -- read() must return None rather than
        raise, same "never raises for a merely-missing frame" contract
        CameraSource.read() documents for every backend."""
        node = rclpy.create_node("test_ros_image_camera_unknown_encoding")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            msg = Image()
            msg.width = 4
            msg.height = 3
            msg.encoding = "yuv422"
            msg.step = 4 * 2
            msg.data = bytes([0]) * (msg.step * 3)
            camera._on_image(msg)

            assert camera.is_connected is True
            assert camera.read() is None
        finally:
            node.destroy_node()

    def test_frame_id_increments_across_reads_of_the_same_cached_message(self):
        node = rclpy.create_node("test_ros_image_camera_frame_id")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            camera._on_image(_make_image_msg(4, 3, "bgr8"))

            f0 = camera.read()
            f1 = camera.read()
            assert [f0.frame_id, f1.frame_id] == [0, 1]
        finally:
            node.destroy_node()

    def test_later_message_replaces_the_cached_frame(self):
        node = rclpy.create_node("test_ros_image_camera_replace")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            camera._on_image(_make_image_msg(4, 3, "bgr8", fill_value=10))
            camera._on_image(_make_image_msg(8, 6, "bgr8", fill_value=20))

            frame = camera.read()
            assert frame.width == 8
            assert frame.height == 6
            assert (frame.image == 20).all()
        finally:
            node.destroy_node()


class TestClose:
    def test_close_destroys_the_subscription_without_raising(self):
        node = rclpy.create_node("test_ros_image_camera_close")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            camera.close()  # must not raise
        finally:
            node.destroy_node()

    def test_close_is_safe_to_call_twice(self):
        node = rclpy.create_node("test_ros_image_camera_close_twice")
        try:
            camera = ROSImageCameraSource(node, topic="/camera/image_raw")
            camera.close()
            camera.close()  # must not raise a second time
        finally:
            node.destroy_node()
