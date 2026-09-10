"""Deterministic, zero-weights, zero-network, zero-GPU PersonDetector.

Used by tests and by camera_backend=synthetic dev-mode by default (via
detector_backend=mock) -- the whole perception pipeline can be exercised
end-to-end with no model, no camera hardware, and no flakiness.
"""
from __future__ import annotations

import uuid
from typing import Optional

from ..camera_source import Frame
from ..detection_types import BBox, Detection
from ..detector import PersonDetector


class MockPersonDetector(PersonDetector):
    def __init__(
        self,
        detections_per_frame: int = 0,
        fixed_boxes: Optional[list[tuple[float, float, float, float, float]]] = None,
    ) -> None:
        """`fixed_boxes`, if given, is a list of (x_min, y_min, x_max, y_max,
        confidence) tuples in absolute pixel coordinates, reproduced verbatim
        on every detect() call (must fit within whatever frame is passed to
        detect()). Otherwise, `detections_per_frame` deterministic boxes,
        sized as a fixed fraction of the frame, are generated on every call."""
        self._detections_per_frame = detections_per_frame
        self._fixed_boxes = fixed_boxes

    def detect(self, frame: Frame) -> list[Detection]:
        if self._fixed_boxes is not None:
            boxes = self._fixed_boxes
        else:
            boxes = [
                (
                    0.1 * frame.width,
                    0.1 * frame.height,
                    0.3 * frame.width,
                    0.3 * frame.height,
                    0.9,
                )
                for _ in range(self._detections_per_frame)
            ]

        detections = []
        for x_min, y_min, x_max, y_max, confidence in boxes:
            bbox = BBox(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max)
            detections.append(
                Detection(
                    detection_id=uuid.uuid4().hex,
                    class_name="person",
                    confidence=confidence,
                    bbox=bbox,
                    frame_width=frame.width,
                    frame_height=frame.height,
                    timestamp=frame.timestamp,
                    source="mock",
                    model_name="mock",
                )
            )
        return detections

    @property
    def ready(self) -> bool:
        return True

    @property
    def name(self) -> str:
        return "mock"
