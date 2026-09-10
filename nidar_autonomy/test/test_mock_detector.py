"""Pure-logic tests for perception/models/mock_detector.py -- no ROS import."""
from nidar_autonomy.perception.camera_source import Frame
from nidar_autonomy.perception.detection_types import Detection
from nidar_autonomy.perception.models.mock_detector import MockPersonDetector


def _frame(width=640, height=480, timestamp=100.0, frame_id=0):
    import numpy as np

    return Frame(
        image=np.zeros((height, width, 3), dtype="uint8"),
        width=width,
        height=height,
        timestamp=timestamp,
        frame_id=frame_id,
    )


class TestMockPersonDetector:
    def test_zero_detections_by_default(self):
        detector = MockPersonDetector()
        detections = detector.detect(_frame())
        assert detections == []

    def test_n_detections(self):
        detector = MockPersonDetector(detections_per_frame=3)
        detections = detector.detect(_frame())
        assert len(detections) == 3
        for d in detections:
            assert isinstance(d, Detection)
            assert d.class_name == "person"
            assert d.source == "mock"

    def test_detections_carry_frame_metadata(self):
        detector = MockPersonDetector(detections_per_frame=1)
        frame = _frame(width=320, height=240, timestamp=42.0)
        (detection,) = detector.detect(frame)
        assert detection.frame_width == 320
        assert detection.frame_height == 240
        assert detection.timestamp == 42.0

    def test_fixed_boxes(self):
        detector = MockPersonDetector(fixed_boxes=[(10.0, 10.0, 50.0, 90.0, 0.75)])
        (detection,) = detector.detect(_frame())
        assert detection.confidence == 0.75
        assert detection.bbox.x_min == 10.0
        assert detection.bbox.y_max == 90.0

    def test_ready_and_name(self):
        detector = MockPersonDetector()
        assert detector.ready is True
        assert detector.name == "mock"
