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


def test_start_while_aborted_is_a_noop():
    """A bare 'start' can never leave 'aborted' by itself -- the only way
    out is a confirmed ground reset (handle_ground_reset_confirmed), which
    is driven by real FCU telemetry via mission_state_node.py, never by
    the /gcs/command channel directly. This is deliberate: re-arming a
    live vehicle should never be a side effect of a stray command."""
    m = MissionStateMachine()
    m.handle_command("abort")
    assert m.handle_command("start") == "aborted"


def test_repeated_start_while_aborted_stays_aborted():
    m = MissionStateMachine()
    m.handle_command("abort")
    m.handle_command("start")
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


def test_arm_failed_while_entering_goes_to_aborted():
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_arm_failed() == "aborted"
    assert m.state == "aborted"


def test_arm_failed_while_idle_is_a_noop():
    m = MissionStateMachine()
    assert m.handle_arm_failed() == "idle"


def test_arm_failed_while_already_aborted_is_a_noop():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    assert m.handle_arm_failed() == "aborted"


def test_ground_reset_confirmed_after_abort_goes_to_idle():
    """The whole point of this task: a completed ground abort must be
    able to make the mission restartable again without restarting the
    node."""
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    assert m.handle_ground_reset_confirmed() == "idle"
    assert m.state == "idle"


def test_start_works_again_after_ground_reset():
    m = MissionStateMachine()
    m.handle_command("start")
    m.handle_command("abort")
    m.handle_ground_reset_confirmed()
    assert m.handle_command("start") == "entering"


def test_ground_reset_confirmed_while_idle_is_a_noop():
    m = MissionStateMachine()
    assert m.handle_ground_reset_confirmed() == "idle"


def test_ground_reset_confirmed_while_entering_is_a_noop():
    """A disarm confirmation must never fabricate a transition out of a
    state other than 'aborted' -- in particular it must never appear to
    "complete" an in-progress arm."""
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_ground_reset_confirmed() == "entering"


def test_unsolicited_fcu_disarm_does_not_reach_idle_by_itself():
    """An unsolicited FCU disarm (e.g. ground-idle auto-disarm) must only
    ever reach 'aborted', never 'idle' -- only a DISARM this node itself
    commanded and then confirmed (handle_ground_reset_confirmed) may do
    that. Reaching 'idle' from here would require a separate, explicit
    ground-reset confirmation, not a side effect of the FCU disarming
    itself."""
    m = MissionStateMachine()
    m.handle_command("start")
    assert m.handle_fcu_disarmed() == "aborted"
    assert m.state == "aborted"


def test_full_lifecycle_start_abort_reset_restart():
    m = MissionStateMachine()
    assert m.state == "idle"
    assert m.handle_command("start") == "entering"
    assert m.handle_command("abort") == "aborted"
    assert m.handle_ground_reset_confirmed() == "idle"
    assert m.handle_command("start") == "entering"
