#!/usr/bin/env python3
"""RobotPy Command-based robot with an MK4i NEO Vortex swerve drivetrain."""

import commands2
import wpilib

from commands.module_test import ModuleTestCommand
from commands.swerve_drive import SwerveDriveCommand
from commands.tune_pid import TuneDrivePID, TuneTurnPID
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

        # Tuning helpers: run these from SmartDashboard to tune one module's
        # drive/steering PID live. They interrupt the default teleop command.
        wpilib.SmartDashboard.putData(
            "Tune/Tune Drive PID", TuneDrivePID(self.swerve)
        )
        wpilib.SmartDashboard.putData(
            "Tune/Tune Turn PID", TuneTurnPID(self.swerve)
        )

        # Manual single-module test: type a speed (m/s) and angle (deg) in
        # SmartDashboard to command one module directly.
        wpilib.SmartDashboard.putData(
            "Test/Module State", ModuleTestCommand(self.swerve)
        )

        print("[robotInit] Swerve drive and teleop command ready")

    def robotPeriodic(self) -> None:
        """Run the command scheduler and publish driver input every loop."""
        commands2.CommandScheduler.getInstance().run()

        # Raw controller axes; confirms the sticks are read and centered.
        wpilib.SmartDashboard.putNumber(
            "Driver/Left X", self.driver_controller.getLeftX()
        )
        wpilib.SmartDashboard.putNumber(
            "Driver/Left Y", self.driver_controller.getLeftY()
        )
        wpilib.SmartDashboard.putNumber(
            "Driver/Right X", self.driver_controller.getRightX()
        )
        wpilib.SmartDashboard.putNumber(
            "Driver/Right Y", self.driver_controller.getRightY()
        )
        wpilib.SmartDashboard.putBoolean(
            "Driver/A Button", self.driver_controller.getAButton()
        )

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
