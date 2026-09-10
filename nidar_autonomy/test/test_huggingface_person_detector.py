"""Tests for perception/models/huggingface_person_detector.py.

huggingface_hub.hf_hub_download and ultralytics.YOLO are fully monkeypatched
-- no real network call and no real model load happens in this suite.
"""
import math

import numpy as np
import pytest

from nidar_autonomy.perception.camera_source import Frame
from nidar_autonomy.perception.models.huggingface_person_detector import (
    HuggingFacePersonDetector,
)


def _frame(width=640, height=480, timestamp=100.0):
    return Frame(
        image=np.zeros((height, width, 3), dtype=np.uint8),
        width=width,
        height=height,
        timestamp=timestamp,
        frame_id=0,
    )


def _make_detector(**overrides):
    kwargs = dict(
        model_repo_id="Ultralytics/YOLO11",
        model_filename="yolo11n.pt",
        model_local_path=None,
        confidence_threshold=0.5,
        allow_download=False,
    )
    kwargs.update(overrides)
    return HuggingFacePersonDetector(**kwargs)


class _FakeBox:
    def __init__(self, cls_id, conf, xyxy):
        self.cls = [cls_id]
        self.conf = [conf]
        self.xyxy = [xyxy]


class _FakeResult:
    def __init__(self, boxes):
        self.boxes = boxes


class _FakeYOLO:
    """Stand-in for ultralytics.YOLO -- records the weights path it was
    "loaded" with and returns a pre-programmed set of results on call."""

    last_results = []

    def __init__(self, weights_path):
        self.weights_path = weights_path

    def __call__(self, image, verbose=False, conf=0.5, imgsz=640):
        return type(self).last_results


class TestReadyBeforeLoad:
    def test_ready_false_before_any_detect_call(self):
        detector = _make_detector()
        assert detector.ready is False

    def test_ready_false_when_hf_hub_download_fails_offline(self, monkeypatch):
        import huggingface_hub

        def _raise(*args, **kwargs):
            raise OSError("offline and not cached")

        monkeypatch.setattr(huggingface_hub, "hf_hub_download", _raise)
        detector = _make_detector(allow_download=False)
        detections = detector.detect(_frame())
        assert detections == []
        assert detector.ready is False


class TestSuccessfulLoadAndDetect:
    def test_detect_filters_to_person_class_only(self, monkeypatch):
        import huggingface_hub
        import ultralytics

        monkeypatch.setattr(
            huggingface_hub, "hf_hub_download", lambda **kwargs: "/fake/weights.pt"
        )
        _FakeYOLO.last_results = [
            _FakeResult(
                [
                    _FakeBox(0, 0.9, [10.0, 10.0, 50.0, 90.0]),  # person -> kept
                    _FakeBox(2, 0.8, [5.0, 5.0, 20.0, 20.0]),  # car -> filtered out
                ]
            )
        ]
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(allow_download=True)
        detections = detector.detect(_frame(width=640, height=480, timestamp=7.0))

        assert detector.ready is True
        assert len(detections) == 1
        d = detections[0]
        assert d.class_name == "person"
        assert d.confidence == pytest.approx(0.9)
        assert d.bbox.x_min == 10.0
        assert d.bbox.y_max == 90.0
        assert d.source == "huggingface"
        assert d.model_name == "Ultralytics/YOLO11/yolo11n.pt"
        assert d.frame_width == 640
        assert d.frame_height == 480
        assert d.timestamp == 7.0

    def test_model_loaded_only_once_across_multiple_detect_calls(self, monkeypatch):
        import huggingface_hub
        import ultralytics

        calls = {"count": 0}

        def _download(**kwargs):
            calls["count"] += 1
            return "/fake/weights.pt"

        monkeypatch.setattr(huggingface_hub, "hf_hub_download", _download)
        _FakeYOLO.last_results = [_FakeResult([])]
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(allow_download=True)
        detector.detect(_frame())
        detector.detect(_frame())
        assert calls["count"] == 1

    def test_local_path_used_directly_without_hf_call(self, monkeypatch, tmp_path):
        import huggingface_hub
        import ultralytics

        weights_file = tmp_path / "weights.pt"
        weights_file.write_bytes(b"fake")

        def _fail_if_called(**kwargs):
            raise AssertionError("hf_hub_download should not be called when model_local_path exists")

        monkeypatch.setattr(huggingface_hub, "hf_hub_download", _fail_if_called)
        _FakeYOLO.last_results = [_FakeResult([])]
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(model_local_path=str(weights_file))
        detector.detect(_frame())
        assert detector.ready is True

    def test_malformed_box_is_skipped_without_crashing_detect(self, monkeypatch):
        import huggingface_hub
        import ultralytics

        monkeypatch.setattr(
            huggingface_hub, "hf_hub_download", lambda **kwargs: "/fake/weights.pt"
        )
        _FakeYOLO.last_results = [
            _FakeResult(
                [
                    _FakeBox(0, 0.9, [math.nan, 10.0, 50.0, 90.0]),  # malformed -> skipped
                    _FakeBox(0, 0.85, [10.0, 10.0, 50.0, 90.0]),  # good -> kept
                ]
            )
        ]
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(allow_download=True)
        detections = detector.detect(_frame())

        assert len(detections) == 1
        assert detections[0].confidence == pytest.approx(0.85)

    def test_name_property(self):
        detector = _make_detector()
        assert detector.name == "huggingface:Ultralytics/YOLO11/yolo11n.pt"

    def test_multiple_person_detections_in_a_single_frame_are_all_returned(self, monkeypatch):
        import huggingface_hub
        import ultralytics

        monkeypatch.setattr(
            huggingface_hub, "hf_hub_download", lambda **kwargs: "/fake/weights.pt"
        )
        _FakeYOLO.last_results = [
            _FakeResult(
                [
                    _FakeBox(0, 0.95, [10.0, 10.0, 50.0, 90.0]),
                    _FakeBox(0, 0.80, [200.0, 15.0, 260.0, 120.0]),
                    _FakeBox(0, 0.60, [300.0, 30.0, 340.0, 130.0]),
                ]
            )
        ]
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(allow_download=True)
        detections = detector.detect(_frame())

        assert len(detections) == 3
        assert all(d.class_name == "person" for d in detections)
        assert sorted(d.confidence for d in detections) == [
            pytest.approx(0.60),
            pytest.approx(0.80),
            pytest.approx(0.95),
        ]
        # every detection gets its own unique id -- not shared/reused
        assert len({d.detection_id for d in detections}) == 3


class TestNoAndMalformedDetections:
    def test_empty_results_list_yields_no_detections(self, monkeypatch):
        import huggingface_hub
        import ultralytics

        monkeypatch.setattr(
            huggingface_hub, "hf_hub_download", lambda **kwargs: "/fake/weights.pt"
        )
        _FakeYOLO.last_results = []
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(allow_download=True)
        assert detector.detect(_frame()) == []

    def test_result_with_boxes_none_is_skipped_without_crashing(self, monkeypatch):
        """Some ultralytics result shapes can have `.boxes is None` (no
        detections at all in that result) -- detect() must skip over that
        result rather than crash iterating it."""
        import huggingface_hub
        import ultralytics

        monkeypatch.setattr(
            huggingface_hub, "hf_hub_download", lambda **kwargs: "/fake/weights.pt"
        )

        class _FakeResultNoBoxes:
            boxes = None

        _FakeYOLO.last_results = [_FakeResultNoBoxes()]
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(allow_download=True)
        assert detector.detect(_frame()) == []

    def test_box_with_empty_cls_list_is_skipped_via_index_error(self, monkeypatch):
        import huggingface_hub
        import ultralytics

        monkeypatch.setattr(
            huggingface_hub, "hf_hub_download", lambda **kwargs: "/fake/weights.pt"
        )

        class _EmptyClsBox:
            cls: list = []
            conf = [0.9]
            xyxy = [[10.0, 10.0, 50.0, 90.0]]

        _FakeYOLO.last_results = [
            _FakeResult([_EmptyClsBox(), _FakeBox(0, 0.7, [1.0, 1.0, 5.0, 5.0])])
        ]
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(allow_download=True)
        detections = detector.detect(_frame())

        assert len(detections) == 1
        assert detections[0].confidence == pytest.approx(0.7)

    def test_out_of_range_confidence_from_model_is_skipped_not_crashed(self, monkeypatch):
        """A confidence outside [0, 1] fails Detection's own validation
        (detection_types.py) -- detect() must catch that and skip the box,
        same as any other malformed output, rather than letting the
        Detection constructor's ValueError propagate out of detect()."""
        import huggingface_hub
        import ultralytics

        monkeypatch.setattr(
            huggingface_hub, "hf_hub_download", lambda **kwargs: "/fake/weights.pt"
        )
        _FakeYOLO.last_results = [
            _FakeResult(
                [
                    _FakeBox(0, 1.5, [10.0, 10.0, 50.0, 90.0]),  # invalid confidence
                    _FakeBox(0, 0.65, [11.0, 11.0, 51.0, 91.0]),
                ]
            )
        ]
        monkeypatch.setattr(ultralytics, "YOLO", _FakeYOLO)

        detector = _make_detector(allow_download=True)
        detections = detector.detect(_frame())

        assert len(detections) == 1
        assert detections[0].confidence == pytest.approx(0.65)
