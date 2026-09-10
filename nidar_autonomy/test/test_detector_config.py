"""Tests for perception/detector_config.py.

The dataclass itself is plain (no ROS dependency) and is always tested.
from_ros_params() is only exercised if rclpy is importable in this
environment -- see test_telemetry_bridge_node.py for the same gating idiom.
"""
import pytest

from nidar_autonomy.perception.detector_config import PerceptionConfig


class TestPerceptionConfigDefaults:
    def test_default_construction(self):
        config = PerceptionConfig()
        assert config.enabled is True
        assert config.detector_backend == "mock"
        assert config.model_repo_id == "Ultralytics/YOLO11"
        assert config.model_filename == "yolo11n.pt"
        assert config.model_local_path is None
        assert config.confidence_threshold == 0.5
        assert config.max_input_size == 640
        assert config.camera_backend == "synthetic"
        assert config.camera_device == "/dev/video0"
        assert config.camera_ros_topic == "/camera/image_raw"
        assert config.frame_width == 640
        assert config.frame_height == 480
        assert config.detection_rate_hz == 2.0
        assert config.capture_rate_hz == 15.0
        assert config.video_stream_host == "0.0.0.0"
        assert config.video_stream_port == 8090
        assert config.allow_model_download is False

    def test_overrides(self):
        config = PerceptionConfig(detector_backend="huggingface", camera_backend="v4l2")
        assert config.detector_backend == "huggingface"
        assert config.camera_backend == "v4l2"


pytest.importorskip("rclpy")

import rclpy  # noqa: E402


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestFromRosParams:
    def test_defaults_round_trip_through_ros_params(self):
        node = rclpy.create_node("test_perception_config_defaults")
        try:
            config = PerceptionConfig.from_ros_params(node)
            assert config == PerceptionConfig()
        finally:
            node.destroy_node()

    def test_overridden_params_are_read_back(self):
        node = rclpy.create_node(
            "test_perception_config_overrides",
            parameter_overrides=[
                rclpy.parameter.Parameter("perception.detector_backend", value="huggingface"),
                rclpy.parameter.Parameter("perception.camera_backend", value="v4l2"),
                rclpy.parameter.Parameter("perception.confidence_threshold", value=0.75),
                rclpy.parameter.Parameter("perception.enabled", value=False),
            ],
        )
        try:
            config = PerceptionConfig.from_ros_params(node)
            assert config.detector_backend == "huggingface"
            assert config.camera_backend == "v4l2"
            assert config.confidence_threshold == pytest.approx(0.75)
            assert config.enabled is False
        finally:
            node.destroy_node()

    def test_model_local_path_unset_stays_none(self):
        node = rclpy.create_node("test_perception_config_local_path_unset")
        try:
            config = PerceptionConfig.from_ros_params(node)
            assert config.model_local_path is None
        finally:
            node.destroy_node()

    def test_model_local_path_set_is_read_back(self):
        node = rclpy.create_node(
            "test_perception_config_local_path_set",
            parameter_overrides=[
                rclpy.parameter.Parameter("perception.model_local_path", value="/tmp/weights.pt"),
            ],
        )
        try:
            config = PerceptionConfig.from_ros_params(node)
            assert config.model_local_path == "/tmp/weights.pt"
        finally:
            node.destroy_node()
