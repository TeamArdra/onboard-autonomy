"""Dev-only PersonDetector stand-in: a pretrained, Hugging-Face-Hub-sourced
YOLO model, filtered to COCO class 0 ("person"). Not a NIDAR-trained model --
this exists purely so the rest of the perception pipeline (camera capture,
Detection contract, GCS topics) can be built and tested end-to-end before a
real, competition-trained model exists. Swapping it out later is exactly the
point of the PersonDetector abstraction (see detector.py).

No network I/O happens at import time. Weight resolution (and the network
call it may imply) happens lazily, the first time it's needed, and only ever
downloads from the Hub if `allow_download` is True -- otherwise
hf_hub_download runs with local_files_only=True and this detector reports
`ready is False` (rather than raising) if the weights aren't already cached.
"""
from __future__ import annotations

import logging
import math
import os
import uuid
from typing import Optional

from ..camera_source import Frame
from ..detection_types import BBox, Detection
from ..detector import PersonDetector

logger = logging.getLogger(__name__)

_COCO_PERSON_CLASS_ID = 0


class HuggingFacePersonDetector(PersonDetector):
    def __init__(
        self,
        model_repo_id: str,
        model_filename: str,
        model_local_path: Optional[str],
        confidence_threshold: float,
        allow_download: bool,
        max_input_size: int = 640,
    ) -> None:
        self.model_repo_id = model_repo_id
        self.model_filename = model_filename
        self.model_local_path = model_local_path
        self.confidence_threshold = confidence_threshold
        self.allow_download = allow_download
        self.max_input_size = max_input_size

        self._model = None
        self._weights_path: Optional[str] = None
        self._load_failed = False

    # ------------------------------------------------------------- loading
    def _resolve_weights_path(self) -> Optional[str]:
        if self.model_local_path and os.path.isfile(self.model_local_path):
            return self.model_local_path

        try:
            from huggingface_hub import hf_hub_download
        except ImportError:
            logger.error(
                "huggingface_hub is not installed; cannot resolve weights for %s/%s",
                self.model_repo_id,
                self.model_filename,
            )
            return None

        try:
            return hf_hub_download(
                repo_id=self.model_repo_id,
                filename=self.model_filename,
                local_files_only=not self.allow_download,
            )
        except Exception as exc:  # noqa: BLE001 -- any Hub/offline/cache failure
            logger.error(
                "Failed to resolve weights for %s/%s (allow_download=%s): %s",
                self.model_repo_id,
                self.model_filename,
                self.allow_download,
                exc,
            )
            return None

    def _ensure_loaded(self) -> None:
        if self._model is not None or self._load_failed:
            return

        weights_path = self._resolve_weights_path()
        if weights_path is None:
            self._load_failed = True
            return

        try:
            from ultralytics import YOLO
        except ImportError:
            logger.error("ultralytics is not installed; cannot load YOLO weights")
            self._load_failed = True
            return

        try:
            self._model = YOLO(weights_path)
            self._weights_path = weights_path
        except Exception as exc:  # noqa: BLE001
            logger.error("Failed to load YOLO weights from %s: %s", weights_path, exc)
            self._load_failed = True

    # ------------------------------------------------------------ detecting
    def detect(self, frame: Frame) -> list[Detection]:
        self._ensure_loaded()
        if self._model is None:
            return []

        results = self._model(
            frame.image,
            verbose=False,
            conf=self.confidence_threshold,
            imgsz=self.max_input_size,
        )

        detections: list[Detection] = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                try:
                    cls_id = int(box.cls[0])
                    if cls_id != _COCO_PERSON_CLASS_ID:
                        continue
                    confidence = float(box.conf[0])
                    x_min, y_min, x_max, y_max = (float(v) for v in box.xyxy[0])
                    if not all(math.isfinite(v) for v in (x_min, y_min, x_max, y_max, confidence)):
                        raise ValueError("non-finite box coordinates/confidence")
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
                            source="huggingface",
                            model_name=f"{self.model_repo_id}/{self.model_filename}",
                        )
                    )
                except (ValueError, TypeError, IndexError) as exc:
                    logger.warning("Skipping malformed detection box: %s", exc)
                    continue

        return detections

    @property
    def ready(self) -> bool:
        return self._model is not None

    @property
    def name(self) -> str:
        return f"huggingface:{self.model_repo_id}/{self.model_filename}"
