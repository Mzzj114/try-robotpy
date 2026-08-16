"""Basic pyfrc tests for the minimal RobotPy project."""

from pyfrc.test_support.controller import TestController


def test_robot_init(control: TestController) -> None:
    """Verify the robot can be initialized without raising exceptions."""
    with control.run_robot():
        # robotInit has been called at this point
        assert control.robot_is_alive


def test_disabled_mode(control: TestController) -> None:
    """Verify the robot enters disabled mode without crashing."""
    with control.run_robot():
        control.step_timing(seconds=1.0, autonomous=False, enabled=False)
        assert control.robot_is_alive


def test_autonomous_mode(control: TestController) -> None:
    """Verify the robot enters autonomous mode without crashing."""
    with control.run_robot():
        control.step_timing(seconds=1.0, autonomous=True, enabled=True)
        assert control.robot_is_alive
