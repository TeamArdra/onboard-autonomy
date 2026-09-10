"""Tests for perception/detector.py's PersonDetector abstract interface --
no ROS import needed. Pins the actual abstract-method contract itself
(rather than just trusting the two concrete subclasses each happen to
implement detect()/ready/name), so a future edit that accidentally drops
an @abc.abstractmethod would be caught here even if nothing else
regressed."""
import pytest

from nidar_autonomy.perception.detector import PersonDetector


def test_person_detector_cannot_be_instantiated_directly():
    with pytest.raises(TypeError):
        PersonDetector()  # type: ignore[abstract]


def test_subclass_missing_detect_cannot_be_instantiated():
    class _MissingDetect(PersonDetector):
        @property
        def ready(self) -> bool:
            return True

        @property
        def name(self) -> str:
            return "incomplete"

    with pytest.raises(TypeError):
        _MissingDetect()  # type: ignore[abstract]


def test_subclass_implementing_every_abstract_member_can_be_instantiated():
    class _Complete(PersonDetector):
        def detect(self, frame):
            return []

        @property
        def ready(self) -> bool:
            return True

        @property
        def name(self) -> str:
            return "complete"

    detector = _Complete()
    assert detector.detect(frame=None) == []
    assert detector.ready is True
    assert detector.name == "complete"
