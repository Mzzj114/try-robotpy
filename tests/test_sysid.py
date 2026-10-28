"""Smoke test for the drive SysId command wiring."""

from typing import TYPE_CHECKING

from commands2.sysid import SysIdRoutine

if TYPE_CHECKING:
    from pyfrc.test_support.controller import TestController
    from robot import Robot


def test_sysid_dynamic_command_starts_and_cancels(
    control: "TestController", robot: "Robot"
) -> None:
    """The prepared dynamic test should schedule and stop when cancelled."""
    with control.run_robot():
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)

        command = robot.sysid.dynamic(SysIdRoutine.Direction.kForward)
        command.schedule()
        control.step_timing(seconds=0.5, autonomous=False, enabled=True)
        assert command.isScheduled()

        command.cancel()
        control.step_timing(seconds=0.1, autonomous=False, enabled=True)
        assert not command.isScheduled()
