"""Camera abstraction: Frame + CameraSource plus three implementations
(synthetic/no-hardware, V4L2/local device, ROS Image topic).

This module must stay importable with no ROS environment sourced -- only
ROSImageCameraSource's __init__ does a local `import rclpy`/`sensor_msgs`,
so a non-ROS test environment can still import and exercise
SyntheticCameraSource / V4L2CameraSource.
"""
from __future__ import annotations

import abc
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class Frame:
    image: np.ndarray  # HxWx3 BGR uint8
    width: int
    height: int
    timestamp: float
    frame_id: int


class CameraSource(abc.ABC):
    @abc.abstractmethod
    def open(self) -> None: ...

    @abc.abstractmethod
    def read(self) -> Optional[Frame]:
        """Returns None if no frame is available / disconnected -- never
        raises for a merely-missing frame."""

    @abc.abstractmethod
    def close(self) -> None: ...

    @property
    @abc.abstractmethod
    def is_connected(self) -> bool: ...


class SyntheticCameraSource(CameraSource):
    """Pure-numpy, no cv2/hardware required. Deterministic given `seed` --
    used as the default dev/test camera backend (camera_backend=synthetic)
    so perception can be exercised with zero hardware and zero flakiness."""

    def __init__(self, width: int = 640, height: int = 480, seed: int = 0) -> None:
        self._width = width
        self._height = height
        self._rng = np.random.default_rng(seed)
        self._frame_id = 0
        self._opened = False

    def open(self) -> None:
        self._opened = True

    def read(self) -> Optional[Frame]:
        if not self._opened:
            return None
        image = self._rng.integers(
            0, 256, size=(self._height, self._width, 3), dtype=np.uint8
        )
        frame = Frame(
            image=image,
            width=self._width,
            height=self._height,
            timestamp=time.time(),
            frame_id=self._frame_id,
        )
        self._frame_id += 1
        return frame

    def close(self) -> None:
        self._opened = False

    @property
    def is_connected(self) -> bool:
        return self._opened


class V4L2CameraSource(CameraSource):
    """Wraps cv2.VideoCapture(device). Never raises on a missing/unopenable
    device -- open() just leaves is_connected False, read() just returns
    None. Status tracking/logging beyond that is the caller's (node's) job."""

    def __init__(self, device: str = "/dev/video0", width: int = 640, height: int = 480) -> None:
        self._device = device
        self._width = width
        self._height = height
        self._cap = None
        self._frame_id = 0

    def open(self) -> None:
        import cv2

        cap = cv2.VideoCapture(self._device)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap = cap

    def read(self) -> Optional[Frame]:
        if self._cap is None or not self._cap.isOpened():
            return None
        ok, image = self._cap.read()
        if not ok or image is None:
            return None
        height, width = image.shape[0], image.shape[1]
        frame = Frame(
            image=image,
            width=width,
            height=height,
            timestamp=time.time(),
            frame_id=self._frame_id,
        )
        self._frame_id += 1
        return frame

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
        self._cap = None

    @property
    def is_connected(self) -> bool:
        return self._cap is not None and self._cap.isOpened()


class ROSImageCameraSource(CameraSource):
    """Subscribes sensor_msgs/Image on an rclpy node, caches the latest
    message, converts to Frame on read(). Manual decode (bgr8/rgb8/mono8) --
    no cv_bridge dependency, keeping this module self-contained."""

    def __init__(self, node, topic: str = "/camera/image_raw") -> None:
        import rclpy  # noqa: F401  (import guard: fail loudly if ROS isn't sourced)
        from sensor_msgs.msg import Image

        self._node = node
        self._topic = topic
        self._latest_msg: Optional[Image] = None
        self._frame_id = 0
        self._subscription = node.create_subscription(Image, topic, self._on_image, 10)

    def _on_image(self, msg) -> None:
        self._latest_msg = msg

    def open(self) -> None:
        pass  # subscription is created in __init__; nothing else to open

    def read(self) -> Optional[Frame]:
        msg = self._latest_msg
        if msg is None:
            return None
        image = self._decode(msg)
        if image is None:
            return None
        frame = Frame(
            image=image,
            width=msg.width,
            height=msg.height,
            timestamp=time.time(),
            frame_id=self._frame_id,
        )
        self._frame_id += 1
        return frame

    @staticmethod
    def _decode(msg) -> Optional[np.ndarray]:
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "bgr8":
            return buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(
                msg.height, msg.width, 3
            )
        if msg.encoding == "rgb8":
            rgb = buf.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(
                msg.height, msg.width, 3
            )
            return rgb[:, :, ::-1]
        if msg.encoding == "mono8":
            mono = buf.reshape(msg.height, msg.step)[:, : msg.width]
            return np.repeat(mono[:, :, np.newaxis], 3, axis=2)
        return None

    def close(self) -> None:
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None

    @property
    def is_connected(self) -> bool:
        return self._latest_msg is not None
