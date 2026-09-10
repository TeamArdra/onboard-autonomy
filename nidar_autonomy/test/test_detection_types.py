"""Pure-logic tests for perception/detection_types.py -- no ROS import."""
import math

import pytest

from nidar_autonomy.perception.detection_types import BBox, Detection


def _bbox(x_min=10.0, y_min=20.0, x_max=50.0, y_max=80.0):
    return BBox(x_min=x_min, y_min=y_min, x_max=x_max, y_max=y_max)


def _detection(**overrides):
    kwargs = dict(
        detection_id="abc123",
        class_name="person",
        confidence=0.8,
        bbox=_bbox(),
        frame_width=640,
        frame_height=480,
        timestamp=1234.5,
    )
    kwargs.update(overrides)
    return Detection(**kwargs)


class TestBBox:
    def test_valid_construction(self):
        b = _bbox(0.0, 0.0, 10.0, 20.0)
        assert b.width == 10.0
        assert b.height == 20.0
        assert b.center_x == 5.0
        assert b.center_y == 10.0

    def test_x_min_must_be_less_than_x_max(self):
        with pytest.raises(ValueError):
            BBox(x_min=10.0, y_min=0.0, x_max=10.0, y_max=20.0)
        with pytest.raises(ValueError):
            BBox(x_min=20.0, y_min=0.0, x_max=10.0, y_max=20.0)

    def test_y_min_must_be_less_than_y_max(self):
        with pytest.raises(ValueError):
            BBox(x_min=0.0, y_min=20.0, x_max=10.0, y_max=20.0)

    def test_negative_coords_rejected(self):
        with pytest.raises(ValueError):
            BBox(x_min=-1.0, y_min=0.0, x_max=10.0, y_max=20.0)
        with pytest.raises(ValueError):
            BBox(x_min=0.0, y_min=-1.0, x_max=10.0, y_max=20.0)

    def test_non_finite_coords_rejected(self):
        with pytest.raises(ValueError):
            BBox(x_min=math.nan, y_min=0.0, x_max=10.0, y_max=20.0)
        with pytest.raises(ValueError):
            BBox(x_min=0.0, y_min=0.0, x_max=math.inf, y_max=20.0)


class TestDetection:
    def test_valid_construction(self):
        d = _detection()
        assert d.center_x == d.bbox.center_x
        assert d.center_y == d.bbox.center_y

    def test_confidence_out_of_range_rejected(self):
        with pytest.raises(ValueError):
            _detection(confidence=1.5)
        with pytest.raises(ValueError):
            _detection(confidence=-0.1)

    def test_confidence_boundaries_allowed(self):
        _detection(confidence=0.0)
        _detection(confidence=1.0)

    def test_non_positive_frame_dims_rejected(self):
        with pytest.raises(ValueError):
            _detection(frame_width=0)
        with pytest.raises(ValueError):
            _detection(frame_height=-1)

    def test_bbox_out_of_frame_rejected(self):
        with pytest.raises(ValueError):
            _detection(bbox=_bbox(x_max=1000.0), frame_width=640, frame_height=480)
        with pytest.raises(ValueError):
            _detection(bbox=_bbox(y_max=1000.0), frame_width=640, frame_height=480)

    def test_bbox_does_not_get_silently_clamped(self):
        # A bbox exactly at the frame edge is fine; one that overflows raises
        # rather than being clamped -- clamping (if ever wanted) is the
        # caller's job, not this constructor's.
        _detection(bbox=_bbox(x_max=640.0, y_max=480.0), frame_width=640, frame_height=480)
        with pytest.raises(ValueError):
            _detection(bbox=_bbox(x_max=640.1), frame_width=640, frame_height=480)

    def test_to_dict_has_exact_keys(self):
        d = _detection(track_id="track-1", source="mock", model_name="mock")
        out = d.to_dict()
        assert set(out.keys()) == {
            "detection_id",
            "class_name",
            "confidence",
            "bbox",
            "frame_width",
            "frame_height",
            "timestamp",
            "track_id",
            "center_x",
            "center_y",
            "source",
            "model_name",
        }
        assert set(out["bbox"].keys()) == {"x_min", "y_min", "x_max", "y_max"}

    def test_to_dict_from_dict_round_trip(self):
        d = _detection(track_id="track-1", source="huggingface", model_name="repo/file.pt")
        round_tripped = Detection.from_dict(d.to_dict())
        assert round_tripped == d

    def test_from_dict_defaults_track_id_and_source(self):
        d = _detection()
        payload = d.to_dict()
        del payload["track_id"]
        del payload["source"]
        del payload["model_name"]
        round_tripped = Detection.from_dict(payload)
        assert round_tripped.track_id is None
        assert round_tripped.source == "unknown"
        assert round_tripped.model_name == "unknown"
