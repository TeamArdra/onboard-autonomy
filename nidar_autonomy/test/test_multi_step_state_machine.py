"""Pure-logic tests for MultiStepTestStateMachine -- no rclpy dependency,
runs anywhere pytest does (same posture as test_frontier_detector.py /
test_geometry.py in this test directory)."""

from nidar_autonomy.flight_test.multi_step_state_machine import MultiStepTestStateMachine


class TestHappyPath:
    def test_full_run_idle_to_complete(self):
        m = MultiStepTestStateMachine()
        assert m.state == "idle"
        assert m.handle_start() == "arming"
        assert m.handle_armed() == "executing"
        assert m.handle_all_steps_done() == "landing"
        assert m.handle_landed() == "complete"


class TestArmFailure:
    def test_arm_failed_goes_to_failed_not_aborted(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        assert m.handle_arm_failed() == "failed"

    def test_arm_failed_noop_outside_arming(self):
        m = MultiStepTestStateMachine()
        assert m.handle_arm_failed() == "idle"


class TestStepFailure:
    def test_step_failed_during_executing_goes_to_failed(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_armed()
        assert m.handle_step_failed() == "failed"

    def test_step_failed_noop_outside_executing(self):
        m = MultiStepTestStateMachine()
        assert m.handle_step_failed() == "idle"


class TestAbort:
    def test_abort_from_arming(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        assert m.handle_abort() == "aborted"

    def test_abort_from_executing(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_armed()
        assert m.handle_abort() == "aborted"

    def test_abort_from_landing(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_armed()
        m.handle_all_steps_done()
        assert m.handle_abort() == "aborted"

    def test_abort_noop_from_idle_complete_aborted_failed(self):
        for setup, expected in [
            (lambda m: None, "idle"),
        ]:
            m = MultiStepTestStateMachine()
            setup(m)
            assert m.handle_abort() == expected

        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_armed()
        m.handle_all_steps_done()
        m.handle_landed()
        assert m.state == "complete"
        assert m.handle_abort() == "complete"

        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_abort()
        assert m.state == "aborted"
        assert m.handle_abort() == "aborted"

        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_arm_failed()
        assert m.state == "failed"
        assert m.handle_abort() == "failed"


class TestRestartability:
    def test_restart_from_complete(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_armed()
        m.handle_all_steps_done()
        m.handle_landed()
        assert m.state == "complete"
        assert m.handle_start() == "arming"

    def test_restart_from_aborted(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_abort()
        assert m.handle_start() == "arming"

    def test_restart_from_failed(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_arm_failed()
        assert m.handle_start() == "arming"

    def test_start_noop_while_in_progress(self):
        m = MultiStepTestStateMachine()
        m.handle_start()
        m.handle_armed()
        assert m.handle_start() == "executing"


class TestDefensiveNoops:
    def test_armed_noop_outside_arming(self):
        m = MultiStepTestStateMachine()
        assert m.handle_armed() == "idle"

    def test_all_steps_done_noop_outside_executing(self):
        m = MultiStepTestStateMachine()
        assert m.handle_all_steps_done() == "idle"

    def test_landed_noop_outside_landing(self):
        m = MultiStepTestStateMachine()
        assert m.handle_landed() == "idle"
