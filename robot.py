#!/usr/bin/env python3
"""RobotPy Command-based robot with a four-wheel differential drivetrain."""

import commands2
import wpilib

from commands.arcade_drive import ArcadeDrive
from subsystems.drivetrain import Drivetrain


class Robot(commands2.TimedCommandRobot):
    def robotInit(self) -> None:
        """Create subsystems, input devices, and default commands."""
        self.drivetrain = Drivetrain()
        self.driver_joystick = wpilib.Joystick(0)

        # Default command: arcade drive from the driver's joystick.
        self.drivetrain.setDefaultCommand(
            ArcadeDrive(
                self.drivetrain,
                forward_supplier=lambda: -self.driver_joystick.getY(),
                rotation_supplier=lambda: self.driver_joystick.getX(),
            )
        )

        print("[robotInit] Drivetrain and arcade drive command ready")

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
