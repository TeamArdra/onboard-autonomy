"""Integration tests for perception/perception_node.py -- requires a real
rclpy context, gated with `pytest.importorskip("rclpy")`, same pattern as
test_telemetry_bridge_node.py. Constructed fully offline/hardware-free
(camera_backend=synthetic, detector_backend=mock are this node's defaults).
"""
import json
import re
from pathlib import Path

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    from nidar_autonomy.perception.perception_node import PerceptionNode

    node = PerceptionNode()
    return node, node.destroy_node


class TestNeverTouchesFlightControl:
    def test_source_has_no_flight_control_imports(self):
        from nidar_autonomy.perception import perception_node

        source = Path(perception_node.__file__).read_text()
        forbidden = ("mavros_msgs", "flight_command", "arming_guard", "mission_state_node")
        for name in forbidden:
            assert not re.search(rf"\b{re.escape(name)}\b", source), (
                f"perception_node.py must never import/reference {name!r}"
            )


class TestCaptureAndDetectionTicks:
    def test_capture_tick_populates_latest_frame_and_stream(self):
        node, cleanup = _make_node()
        try:
            assert node._latest_frame is None
            node._on_capture_tick()
            assert node._latest_frame is not None
            assert node._latest_frame.width == node.config.frame_width
            assert node._latest_frame.height == node.config.frame_height
        finally:
            cleanup()

    def test_detection_tick_publishes_well_formed_json(self):
        node, cleanup = _make_node()
        try:
            node._on_capture_tick()

            captured = {}
            node._detections_pub.publish = lambda msg: captured.setdefault("out", msg)
            node._on_detection_tick()

            payload = json.loads(captured["out"].data)
            assert payload["schema_version"] == 1
            assert payload["frame_width"] == node.config.frame_width
            assert payload["frame_height"] == node.config.frame_height
            assert isinstance(payload["detections"], list)
        finally:
            cleanup()

    def test_person_count_matches_mock_detector_configured_output(self):
        from nidar_autonomy.perception.models.mock_detector import MockPersonDetector

        node, cleanup = _make_node()
        try:
            node._detector = MockPersonDetector(detections_per_frame=3)
            node._on_capture_tick()

            captured = {}
            node._detections_pub.publish = lambda msg: captured.setdefault("out", msg)
            node._on_detection_tick()

            payload = json.loads(captured["out"].data)
            assert len(payload["detections"]) == 3
            assert node._last_person_count == 3

            status_captured = {}
            node._status_pub.publish = lambda msg: status_captured.setdefault("out", msg)
            node._on_status_tick()
            status = json.loads(status_captured["out"].data)
            assert status["person_count"] == 3
        finally:
            cleanup()

    def test_detection_tick_with_no_frame_yet_does_not_publish(self):
        node, cleanup = _make_node()
        try:
            captured = {}
            node._detections_pub.publish = lambda msg: captured.setdefault("out", msg)
            node._on_detection_tick()
            assert "out" not in captured
        finally:
            cleanup()

    def test_detector_exception_is_caught_and_does_not_crash_node(self):
        node, cleanup = _make_node()
        try:
            node._on_capture_tick()

            class _ExplodingDetector:
                ready = True
                name = "exploding"

                def detect(self, frame):
                    raise RuntimeError("boom")

            node._detector = _ExplodingDetector()

            captured = {}
            node._detections_pub.publish = lambda msg: captured.setdefault("out", msg)
            node._on_detection_tick()  # must not raise

            payload = json.loads(captured["out"].data)
            assert payload["detections"] == []
        finally:
            cleanup()


class TestBuildDetectorConfiguration:
    """Proves the actual "model replacement" swap point: _build_detector()
    is the only place perception_node.py picks a PersonDetector subclass,
    driven entirely by PerceptionConfig.detector_backend -- these tests
    exercise that function directly, not through a full node, so a broken
    swap would fail here even if nothing else about node behavior changed."""

    def test_mock_backend_builds_mock_detector(self):
        from nidar_autonomy.perception.detector_config import PerceptionConfig
        from nidar_autonomy.perception.models.mock_detector import MockPersonDetector
        from nidar_autonomy.perception.perception_node import _build_detector

        detector = _build_detector(PerceptionConfig(detector_backend="mock"))
        assert isinstance(detector, MockPersonDetector)

    def test_huggingface_backend_builds_huggingface_detector_wired_to_config_values(self):
        from nidar_autonomy.perception.detector_config import PerceptionConfig
        from nidar_autonomy.perception.models.huggingface_person_detector import (
            HuggingFacePersonDetector,
        )
        from nidar_autonomy.perception.perception_node import _build_detector

        config = PerceptionConfig(
            detector_backend="huggingface",
            model_repo_id="SomeOrg/SomeModel",
            model_filename="weights.pt",
            model_local_path="/tmp/weights.pt",
            confidence_threshold=0.42,
            allow_model_download=True,
            max_input_size=320,
        )
        detector = _build_detector(config)

        assert isinstance(detector, HuggingFacePersonDetector)
        assert detector.model_repo_id == "SomeOrg/SomeModel"
        assert detector.model_filename == "weights.pt"
        assert detector.model_local_path == "/tmp/weights.pt"
        assert detector.confidence_threshold == pytest.approx(0.42)
        assert detector.allow_download is True
        assert detector.max_input_size == 320

    def test_switching_backend_changes_only_the_detector_class_nothing_else_in_config(self):
        """The actual "swapping detector_backend changes which detector
        class is used, without touching anything else" contract: build
        twice from configs that differ ONLY in detector_backend, and
        confirm every other config value that flows into the detector is
        unaffected by which branch was taken."""
        from nidar_autonomy.perception.detector_config import PerceptionConfig
        from nidar_autonomy.perception.models.huggingface_person_detector import (
            HuggingFacePersonDetector,
        )
        from nidar_autonomy.perception.models.mock_detector import MockPersonDetector
        from nidar_autonomy.perception.perception_node import _build_detector

        shared_kwargs = dict(
            model_repo_id="Ultralytics/YOLO11",
            model_filename="yolo11n.pt",
            confidence_threshold=0.66,
        )
        mock_detector = _build_detector(PerceptionConfig(detector_backend="mock", **shared_kwargs))
        hf_detector = _build_detector(
            PerceptionConfig(detector_backend="huggingface", **shared_kwargs)
        )

        assert isinstance(mock_detector, MockPersonDetector)
        assert isinstance(hf_detector, HuggingFacePersonDetector)
        assert hf_detector.confidence_threshold == pytest.approx(0.66)
        assert hf_detector.model_repo_id == "Ultralytics/YOLO11"

    def test_disabled_config_builds_no_detector_regardless_of_backend(self):
        from nidar_autonomy.perception.detector_config import PerceptionConfig
        from nidar_autonomy.perception.perception_node import _build_detector

        assert _build_detector(PerceptionConfig(enabled=False, detector_backend="mock")) is None
        assert _build_detector(PerceptionConfig(enabled=False, detector_backend="huggingface")) is None


class TestBuildCameraConfiguration:
    """Same swap-point guarantee as TestBuildDetectorConfiguration, for
    _build_camera() / PerceptionConfig.camera_backend."""

    def test_synthetic_backend_builds_synthetic_camera_with_configured_dimensions(self):
        from nidar_autonomy.perception.camera_source import SyntheticCameraSource
        from nidar_autonomy.perception.detector_config import PerceptionConfig
        from nidar_autonomy.perception.perception_node import _build_camera

        config = PerceptionConfig(camera_backend="synthetic", frame_width=320, frame_height=240)
        camera = _build_camera(config, node=None)

        assert isinstance(camera, SyntheticCameraSource)
        camera.open()
        frame = camera.read()
        assert frame.width == 320
        assert frame.height == 240

    def test_v4l2_backend_builds_v4l2_camera_with_configured_device(self):
        from nidar_autonomy.perception.camera_source import V4L2CameraSource
        from nidar_autonomy.perception.detector_config import PerceptionConfig
        from nidar_autonomy.perception.perception_node import _build_camera

        config = PerceptionConfig(camera_backend="v4l2", camera_device="/dev/video7")
        camera = _build_camera(config, node=None)

        assert isinstance(camera, V4L2CameraSource)
        assert camera._device == "/dev/video7"

    def test_ros_image_backend_builds_ros_image_camera_on_the_configured_topic(self):
        from nidar_autonomy.perception.camera_source import ROSImageCameraSource
        from nidar_autonomy.perception.detector_config import PerceptionConfig
        from nidar_autonomy.perception.perception_node import _build_camera

        node = rclpy.create_node("test_build_camera_ros_image")
        try:
            config = PerceptionConfig(camera_backend="ros_image", camera_ros_topic="/foo/image")
            camera = _build_camera(config, node)

            assert isinstance(camera, ROSImageCameraSource)
            assert camera._topic == "/foo/image"
        finally:
            node.destroy_node()

    def test_unrecognized_backend_falls_back_to_synthetic(self):
        """Pins actual behavior of the else-branch in _build_camera(): any
        camera_backend value other than "v4l2"/"ros_image" -- including a
        typo -- silently becomes SyntheticCameraSource, same as the
        detector side's fallback-to-mock. Not necessarily desirable, but
        this is the real, current contract."""
        from nidar_autonomy.perception.camera_source import SyntheticCameraSource
        from nidar_autonomy.perception.detector_config import PerceptionConfig
        from nidar_autonomy.perception.perception_node import _build_camera

        config = PerceptionConfig(camera_backend="not_a_real_backend")
        camera = _build_camera(config, node=None)

        assert isinstance(camera, SyntheticCameraSource)


class TestDetectorDisabledAtNodeLevel:
    def test_status_tick_reports_not_ready_and_no_model_name_when_detector_is_none(self):
        node, cleanup = _make_node()
        try:
            node._detector = None

            captured = {}
            node._status_pub.publish = lambda msg: captured.setdefault("out", msg)
            node._on_status_tick()

            status = json.loads(captured["out"].data)
            assert status["detector_ready"] is False
            assert status["model_name"] is None
        finally:
            cleanup()

    def test_detection_tick_with_no_detector_does_not_publish(self):
        node, cleanup = _make_node()
        try:
            node._on_capture_tick()
            node._detector = None

            captured = {}
            node._detections_pub.publish = lambda msg: captured.setdefault("out", msg)
            node._on_detection_tick()

            assert "out" not in captured
        finally:
            cleanup()


class TestStatusTick:
    def test_status_tick_publishes_well_formed_json(self):
        node, cleanup = _make_node()
        try:
            captured = {}
            node._status_pub.publish = lambda msg: captured.setdefault("out", msg)
            node._on_status_tick()

            status = json.loads(captured["out"].data)
            assert status["schema_version"] == 1
            assert status["camera_connected"] is True
            assert status["detector_enabled"] is True
            assert status["detector_backend"] == "mock"
            assert status["model_name"] == "mock"
            assert status["person_count"] == 0
            assert status["last_detection_age_s"] is None
            assert status["video_stream_url"].startswith("http://")
            assert status["video_stream_url"].endswith("/stream.mjpg")
        finally:
            cleanup()

    def test_last_detection_age_s_set_after_a_detection_publish(self):
        node, cleanup = _make_node()
        try:
            node._on_capture_tick()
            node._detections_pub.publish = lambda msg: None
            node._on_detection_tick()

            captured = {}
            node._status_pub.publish = lambda msg: captured.setdefault("out", msg)
            node._on_status_tick()

            status = json.loads(captured["out"].data)
            assert status["last_detection_age_s"] is not None
            assert status["last_detection_age_s"] >= 0.0
        finally:
            cleanup()
