"""Temporary diagnostic test for PathPlanner auto tracking."""

from typing import TYPE_CHECKING

from wpimath.geometry import Pose2d, Rotation2d

if TYPE_CHECKING:
    from pyfrc.test_support.controller import TestController
    from robot import Robot


def test_debug_auto_tracking(
    control: "TestController", robot: "Robot"
) -> None:
    with control.run_robot():
        robot.swerve.resetPose(Pose2d(0.0, 0.0, Rotation2d()))
        for i in range(40):
            control.step_timing(seconds=0.1, autonomous=True, enabled=True)
            pose = robot.swerve.getPose()
            desired = robot.swerve.getDesiredChassisSpeeds()
            measured = robot.swerve.getRobotRelativeSpeeds()
            print(
                f"t={i * 0.1:4.1f} pose=({pose.x:5.2f},{pose.y:5.2f}) "
                f"cmd=({desired.vx:5.2f},{desired.vy:5.2f},{desired.omega:5.2f}) "
                f"meas=({measured.vx:5.2f},{measured.vy:5.2f},{measured.omega:5.2f})"
            )
