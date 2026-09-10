"""PerceptionConfig: plain dataclass of every perception tunable, plus a
from_ros_params() classmethod that declares/reads them as ROS params under
the `perception.*` namespace -- same declare_parameter/get_parameter idiom
as telemetry_bridge_node.py / coverage_tracker_node.py.

The dataclass itself has no ROS dependency -- only from_ros_params() touches
`node`, so this module stays importable (and its defaults testable) with no
ROS environment sourced.
"""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
from typing import Optional


@dataclass
class PerceptionConfig:
    enabled: bool = True
    detector_backend: str = "mock"  # "mock" | "huggingface"
    model_repo_id: str = "Ultralytics/YOLO11"
    model_filename: str = "yolo11n.pt"
    model_local_path: Optional[str] = None  # if set, load directly from disk, no HF calls at all
    confidence_threshold: float = 0.5
    max_input_size: int = 640
    camera_backend: str = "synthetic"  # "synthetic" | "v4l2" | "ros_image"
    camera_device: str = "/dev/video0"
    camera_ros_topic: str = "/camera/image_raw"
    frame_width: int = 640
    frame_height: int = 480
    detection_rate_hz: float = 2.0
    capture_rate_hz: float = 15.0
    video_stream_host: str = "0.0.0.0"
    video_stream_port: int = 8090
    allow_model_download: bool = False  # if False, HF loader must use local_files_only=True

    @classmethod
    def from_ros_params(cls, node) -> "PerceptionConfig":
        defaults = cls()
        kwargs = {}
        for field in dataclasses.fields(defaults):
            param_name = f"perception.{field.name}"
            default_value = getattr(defaults, field.name)
            # rclpy can't infer a type from a None default (declare_parameter
            # warns/rejects it) -- represent "unset" as "" at the ROS-param
            # level for model_local_path, and translate back to None here.
            declared_default = "" if default_value is None else default_value
            node.declare_parameter(param_name, declared_default)
            value = node.get_parameter(param_name).value
            kwargs[field.name] = None if (default_value is None and value == "") else value
        return cls(**kwargs)
