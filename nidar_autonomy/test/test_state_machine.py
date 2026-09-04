import pytest

from nidar_autonomy.state_machine import InvalidCommandError, MissionStateMachine


def test_starts_idle():
    assert MissionStateMachine().state == "idle"


def test_start_from_idle_goes_to_entering():
    m = MissionStateMachine()
    assert m.handle_command("start") == "entering"
    assert m.state == "entering"


def test_start_when_not_idle_is_a_noop():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_command("start") == "entering"  # unchanged, not re-triggered


def test_abort_from_idle_goes_to_aborted():
    m = MissionStateMachine()
    assert m.handle_command("abort") == "aborted"


def test_abort_from_entering_goes_to_aborted():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_command("abort") == "aborted"


def test_abort_after_abort_stays_aborted():
    m = MissionStateMachine()
    m.handle_command("abort")
    assert m.handle_command("abort") == "aborted"


def test_start_after_abort_does_not_restart():
    """Once aborted, a mission cannot be silently restarted by another
    'start' -- state only leaves 'aborted' via whatever out-of-band
    reset procedure a human operator performs between missions, not via
    the /gcs/command channel. This is deliberate: re-arming a live
    vehicle should never be a side effect of a stray command."""
    m = MissionStateMachine()
    m.handle_command("abort")
    assert m.handle_command("start") == "aborted"


@pytest.mark.parametrize("bad_command", ["", "STOP", "waypoint", "abort ", None])
def test_invalid_command_raises(bad_command):
    m = MissionStateMachine()
    with pytest.raises(InvalidCommandError):
        m.handle_command(bad_command)


def test_fcu_disarm_while_entering_goes_to_aborted():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_fcu_disarmed() == "aborted"
    assert m.state == "aborted"


def test_fcu_disarm_while_idle_is_a_noop():
    m = MissionStateMachine()
    assert m.handle_fcu_disarmed() == "idle"


def test_fcu_disarm_while_already_aborted_is_a_noop():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    assert m.handle_fcu_disarmed() == "aborted"
