"""PersonDetector: the swap point between camera frames and normalized
Detection objects.

perception_node.py, and anything downstream of it (the GCS/transport), only
ever talks to this abstract interface -- never to a specific model backend.
Swapping the dev-only Hugging-Face-sourced YOLO stand-in for a future
NIDAR-trained model is a matter of adding a new PersonDetector subclass and
a detector_backend config value, with zero changes anywhere else.
"""
from __future__ import annotations

import abc

from .camera_source import Frame
from .detection_types import Detection


class PersonDetector(abc.ABC):
    @abc.abstractmethod
    def detect(self, frame: Frame) -> list[Detection]: ...

    @property
    @abc.abstractmethod
    def ready(self) -> bool: ...

    @property
    @abc.abstractmethod
    def name(self) -> str: ...
