"""Normalized detection contract shared by every detector backend and by
perception_node.py's published JSON -- see Detection.to_dict()/from_dict().

BINDING CONTRACT: custom-gcs's backend and frontend are being built in
parallel against these exact field names (see the perception task brief).
Do not rename, add, or remove fields without a joint decision with whoever
owns custom-gcs, same rule as the Drone <-> GCS interface in this repo's
CLAUDE.md.
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Optional


def _finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


@dataclass(frozen=True)
class BBox:
    x_min: float
    y_min: float
    x_max: float
    y_max: float

    def __post_init__(self) -> None:
        if not _finite(self.x_min, self.y_min, self.x_max, self.y_max):
            raise ValueError(f"BBox coordinates must all be finite: {self}")
        if self.x_min < 0 or self.y_min < 0:
            raise ValueError(f"BBox coordinates must be >= 0: {self}")
        if self.x_min >= self.x_max:
            raise ValueError(f"BBox x_min ({self.x_min}) must be < x_max ({self.x_max})")
        if self.y_min >= self.y_max:
            raise ValueError(f"BBox y_min ({self.y_min}) must be < y_max ({self.y_max})")

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def height(self) -> float:
        return self.y_max - self.y_min

    @property
    def center_x(self) -> float:
        return (self.x_min + self.x_max) / 2.0

    @property
    def center_y(self) -> float:
        return (self.y_min + self.y_max) / 2.0


@dataclass(frozen=True)
class Detection:
    detection_id: str
    class_name: str
    confidence: float
    bbox: BBox
    frame_width: int
    frame_height: int
    timestamp: float
    track_id: Optional[str] = None
    source: str = "unknown"
    model_name: str = "unknown"

    def __post_init__(self) -> None:
        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError(f"confidence must be within [0, 1], got {self.confidence}")
        if self.frame_width <= 0:
            raise ValueError(f"frame_width must be > 0, got {self.frame_width}")
        if self.frame_height <= 0:
            raise ValueError(f"frame_height must be > 0, got {self.frame_height}")
        if (
            self.bbox.x_min < 0
            or self.bbox.y_min < 0
            or self.bbox.x_max > self.frame_width
            or self.bbox.y_max > self.frame_height
        ):
            raise ValueError(
                f"bbox {self.bbox} does not fit within frame "
                f"[0, {self.frame_width}] x [0, {self.frame_height}]"
            )

    @property
    def center_x(self) -> float:
        return self.bbox.center_x

    @property
    def center_y(self) -> float:
        return self.bbox.center_y

    def to_dict(self) -> dict[str, Any]:
        return {
            "detection_id": self.detection_id,
            "class_name": self.class_name,
            "confidence": self.confidence,
            "bbox": asdict(self.bbox),
            "frame_width": self.frame_width,
            "frame_height": self.frame_height,
            "timestamp": self.timestamp,
            "track_id": self.track_id,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "source": self.source,
            "model_name": self.model_name,
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "Detection":
        bbox = BBox(
            x_min=d["bbox"]["x_min"],
            y_min=d["bbox"]["y_min"],
            x_max=d["bbox"]["x_max"],
            y_max=d["bbox"]["y_max"],
        )
        return cls(
            detection_id=d["detection_id"],
            class_name=d["class_name"],
            confidence=d["confidence"],
            bbox=bbox,
            frame_width=d["frame_width"],
            frame_height=d["frame_height"],
            timestamp=d["timestamp"],
            track_id=d.get("track_id"),
            source=d.get("source", "unknown"),
            model_name=d.get("model_name", "unknown"),
        )
