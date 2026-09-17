#!/usr/bin/env python3
"""RobotPy Command-based robot with an MK4i NEO Vortex swerve drivetrain."""

import commands2
import wpilib

from commands.swerve_drive import SwerveDriveCommand
from constants import OIConstants
from subsystems.swerve_drive import SwerveDrive


class Robot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Create subsystems, input devices, and default commands."""
        self.swerve = SwerveDrive()
        self.driver_controller = wpilib.XboxController(
            OIConstants.kDriverControllerPort
        )

        # Default command: field-oriented swerve drive from the driver's Xbox controller.
        # Left stick drives translation; right stick X rotates the chassis.
        self.swerve.setDefaultCommand(
            SwerveDriveCommand(
                self.swerve,
                x_supplier=lambda: -self.driver_controller.getLeftY(),
                y_supplier=lambda: -self.driver_controller.getLeftX(),
                rot_supplier=lambda: -self.driver_controller.getRightX(),
                field_relative=True,
            )
        )

        print("[robotInit] Swerve drive and teleop command ready")

    def robotPeriodic(self) -> None:
        """Run the command scheduler every loop."""
        commands2.CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        print("[disabledInit]")

    def disabledPeriodic(self) -> None:
        pass

    def autonomousInit(self) -> None:
        print("[autonomousInit]")

    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        print("[teleopInit]")

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        print("[testInit]")

    def testPeriodic(self) -> None:
        pass
