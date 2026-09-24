"""Simulation-backed tests for the swerve drivetrain.

These run the real robot code under pyfrc's simulated HAL, so they exercise
the Spark MAX / CANcoder / NavX wrappers without physical hardware. Physics
support in physics.py makes odometry and the gyro track the commanded motion.
"""

import math
from typing import TYPE_CHECKING

import pytest
import wpilib.simulation
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import SwerveModuleState

if TYPE_CHECKING:
    from pyfrc.test_support.controller import TestController
    from robot import Robot


def _point_module_forward(module) -> None:
    """Force the module's simulated CANcoder to read 0 rad."""
    coder_sim = module.get_can_coder().sim_state
    coder_sim.set_raw_position(0.0)
    coder_sim.set_velocity(0.0)


def test_get_rotation2d_negates_navx_clockwise_yaw(
    control: "TestController", robot: "Robot"
) -> None:
    """NavX yaw is clockwise-positive; the robot must report CCW-positive."""
    with control.run_robot():
        gyro_sim = wpilib.simulation.SimDeviceSim(
            "navX-Sensor", robot.swerve.getGyro().getPort()
        )
        gyro_sim.getDouble("Yaw").set(90.0)
        # Disabled stepping lets the simulated sensor publish the new value
        # without physics.py overwriting it (physics only runs when enabled).
        control.step_timing(seconds=0.5, autonomous=False, enabled=False)

        assert robot.swerve.getRotation2d().degrees() == pytest.approx(
            -90.0, abs=0.1
        )


def test_set_desired_state_takes_shortest_path(
    control: "TestController", robot: "Robot"
) -> None:
    """A target ~180 deg away flips the wheel and negates speed."""
    with control.run_robot():
        module = robot.swerve.getModules()[0]
        _point_module_forward(module)

        module.setDesiredState(
            SwerveModuleState(1.0, Rotation2d.fromDegrees(179.0))
        )

        # getGoal() is the optimized target; getSetpoint() would only be the
        # first profiled step toward it.
        turn_goal = module.get_turn_pid_controller().getGoal().position
        assert math.degrees(turn_goal) == pytest.approx(-1.0, abs=0.5)
        assert module.get_drive_pid_controller().getSetpoint() == pytest.approx(
            -1.0, abs=0.01
        )


def test_set_desired_state_scales_speed_by_cosine(
    control: "TestController", robot: "Robot"
) -> None:
    """A 60 deg steering error keeps cos(60) = 50% of the commanded speed."""
    with control.run_robot():
        module = robot.swerve.getModules()[0]
        _point_module_forward(module)

        module.setDesiredState(
            SwerveModuleState(1.0, Rotation2d.fromDegrees(60.0))
        )

        assert module.get_drive_pid_controller().getSetpoint() == pytest.approx(
            0.5, abs=1e-3
        )


def test_odometry_follows_physics_forward(
    control: "TestController", robot: "Robot"
) -> None:
    """A full-forward stick drives the robot straight along +x."""
    with control.run_robot():
        robot.swerve.resetPose(Pose2d())
        controller_sim = wpilib.simulation.XboxControllerSim(
            robot.driver_controller
        )
        # The robot negates the stick axes, so -1 on Y means "forward".
        controller_sim.setLeftY(-1.0)
        controller_sim.setLeftX(0.0)
        controller_sim.setRightX(0.0)

        control.step_timing(seconds=1.0, autonomous=False, enabled=True)

        pose = robot.swerve.getPose()
        assert pose.x > 1.0
        assert pose.y == pytest.approx(0.0, abs=0.05)
        assert pose.rotation().degrees() == pytest.approx(0.0, abs=1.0)


def test_odometry_follows_physics_rotation(
    control: "TestController", robot: "Robot"
) -> None:
    """A full-left rotation stick spins the robot CCW (positive heading)."""
    with control.run_robot():
        robot.swerve.resetPose(Pose2d())
        controller_sim = wpilib.simulation.XboxControllerSim(
            robot.driver_controller
        )
        controller_sim.setLeftY(0.0)
        controller_sim.setLeftX(0.0)
        controller_sim.setRightX(-1.0)  # robot negates -> positive CCW

        control.step_timing(seconds=1.0, autonomous=False, enabled=True)

        assert robot.swerve.getPose().rotation().degrees() > 30.0
