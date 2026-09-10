"""READ-ONLY PERCEPTION OBSERVER -- captures camera frames, runs a
pluggable person detector, publishes normalized detections and status.
Makes no flight decisions, sends no commands, never touches mavros,
mission_state, or any arming/flight-control code.

Camera capture, detection, and status publication run on three independent
timers (decoupled cadences, same "timer decoupled from input arrival"
pattern as the rest of this repo's read-only observer nodes -- see
telemetry_bridge_node.py / coverage_tracker_node.py). A detector exception
is caught and logged, never allowed to crash this node -- see
_run_detection().

Publishes PERCEPTION_DETECTIONS_TOPIC ("/perception/detections") and
PERCEPTION_STATUS_TOPIC ("/perception/status") as std_msgs/String JSON
blobs, same "normalized JSON blob for the GCS" convention as
telemetry_bridge_node.py's /telemetry/state. Does NOT touch
/vision/survivors -- that is a separate, still-unimplemented,
localized/confirmed-survivor contract owned elsewhere.
"""
from __future__ import annotations

import json
import time
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ..topics import PERCEPTION_DETECTIONS_TOPIC, PERCEPTION_STATUS_TOPIC
from .camera_source import CameraSource, Frame, ROSImageCameraSource, SyntheticCameraSource, V4L2CameraSource
from .detector import PersonDetector
from .detector_config import PerceptionConfig
from .models.huggingface_person_detector import HuggingFacePersonDetector
from .models.mock_detector import MockPersonDetector
from .video_stream import MJPEGStreamServer

SCHEMA_VERSION = 1


def _build_camera(config: PerceptionConfig, node: Node) -> CameraSource:
    if config.camera_backend == "v4l2":
        return V4L2CameraSource(
            device=config.camera_device, width=config.frame_width, height=config.frame_height
        )
    if config.camera_backend == "ros_image":
        return ROSImageCameraSource(node, topic=config.camera_ros_topic)
    return SyntheticCameraSource(width=config.frame_width, height=config.frame_height)


def _build_detector(config: PerceptionConfig) -> Optional[PersonDetector]:
    if not config.enabled:
        return None
    if config.detector_backend == "huggingface":
        return HuggingFacePersonDetector(
            model_repo_id=config.model_repo_id,
            model_filename=config.model_filename,
            model_local_path=config.model_local_path,
            confidence_threshold=config.confidence_threshold,
            allow_download=config.allow_model_download,
            max_input_size=config.max_input_size,
        )
    return MockPersonDetector()


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")

        self.config = PerceptionConfig.from_ros_params(self)
        self._camera = _build_camera(self.config, self)
        self._camera.open()
        self._detector = _build_detector(self.config)
        self._video_server = MJPEGStreamServer(
            host=self.config.video_stream_host, port=self.config.video_stream_port
        )
        self._video_server.start()

        self._latest_frame: Optional[Frame] = None
        self._last_person_count = 0
        self._last_detection_time: Optional[float] = None
        self._capture_count = 0
        self._capture_window_start = time.time()
        self._fps_estimate = 0.0

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        self._detections_pub = self.create_publisher(
            String, PERCEPTION_DETECTIONS_TOPIC, status_qos
        )
        self._status_pub = self.create_publisher(String, PERCEPTION_STATUS_TOPIC, status_qos)

        self.create_timer(1.0 / self.config.capture_rate_hz, self._on_capture_tick)
        self.create_timer(1.0 / self.config.detection_rate_hz, self._on_detection_tick)
        self.create_timer(1.0, self._on_status_tick)

        self.get_logger().info(
            f"perception_node: camera={self.config.camera_backend} "
            f"detector={self.config.detector_backend if self.config.enabled else 'disabled'} "
            f"stream=http://{self.config.video_stream_host}:{self.config.video_stream_port}/stream.mjpg"
        )

    # ------------------------------------------------------------- capture
    def _on_capture_tick(self) -> None:
        try:
            frame = self._camera.read()
        except Exception as exc:  # noqa: BLE001 -- a camera failure must not kill this node
            self.get_logger().warning(f"camera read failed: {exc}")
            frame = None

        if frame is None:
            return

        self._latest_frame = frame
        self._capture_count += 1
        now = time.time()
        elapsed = now - self._capture_window_start
        if elapsed >= 1.0:
            self._fps_estimate = self._capture_count / elapsed
            self._capture_count = 0
            self._capture_window_start = now

        try:
            ok, jpeg = cv2.imencode(".jpg", frame.image)
            if ok:
                self._video_server.update_frame(jpeg.tobytes())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"JPEG encode failed: {exc}")

    # ----------------------------------------------------------- detection
    def _on_detection_tick(self) -> None:
        if self._detector is None or self._latest_frame is None:
            return

        frame = self._latest_frame
        try:
            detections = self._detector.detect(frame)
        except Exception as exc:  # noqa: BLE001 -- detector failures must not crash this node
            self.get_logger().warning(f"detector.detect() failed: {exc}")
            detections = []

        self._last_person_count = len(detections)
        self._last_detection_time = time.time()

        payload = {
            "schema_version": SCHEMA_VERSION,
            "frame_width": frame.width,
            "frame_height": frame.height,
            "timestamp": frame.timestamp,
            "detections": [d.to_dict() for d in detections],
        }
        self._detections_pub.publish(String(data=json.dumps(payload)))

    # -------------------------------------------------------------- status
    def _on_status_tick(self) -> None:
        last_detection_age_s = (
            None if self._last_detection_time is None else time.time() - self._last_detection_time
        )
        payload = {
            "schema_version": SCHEMA_VERSION,
            "camera_connected": self._camera.is_connected,
            "detector_enabled": self.config.enabled,
            "detector_ready": (self._detector.ready if self._detector is not None else False),
            "detector_backend": self.config.detector_backend,
            "model_name": (self._detector.name if self._detector is not None else None),
            "person_count": self._last_person_count,
            "fps": self._fps_estimate,
            "frame_width": self.config.frame_width,
            "frame_height": self.config.frame_height,
            "video_stream_url": (
                f"http://{self.config.video_stream_host}:{self.config.video_stream_port}/stream.mjpg"
            ),
            "last_detection_age_s": last_detection_age_s,
            "timestamp": time.time(),
        }
        self._status_pub.publish(String(data=json.dumps(payload)))

    # ------------------------------------------------------------- cleanup
    def destroy_node(self) -> bool:
        self._video_server.stop()
        self._camera.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
