"""Manual single-module test command.

Reads a desired drive speed and steering angle from SmartDashboard and commands
one swerve module directly, bypassing the chassis kinematics. Useful for
bringing up wiring, steering direction, and encoder offsets on a single module
without the other three moving.

Run it from SmartDashboard under ``Test/``; it requires the swerve subsystem,
so the default teleop command is interrupted while it runs.
"""

import commands2
import wpilib
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from subsystems.swerve_drive import SwerveDrive


class ModuleTestCommand(commands2.Command):
    """Drive one module from dashboard speed (m/s) and angle (deg) values."""

    def __init__(self, swerve: SwerveDrive, module_index: int = 0) -> None:
        """Construct the single-module test command.

        :param swerve: The swerve drivetrain subsystem.
        :param module_index: Module to command, matching kModuleLocations
            order (0 = front left).
        """
        super().__init__()
        self._swerve = swerve
        self._module = swerve.getModules()[module_index]
        self._name = swerve.getModuleNames()[module_index]

        self._key_speed = f"Test/{self._name} Speed (m/s)"
        self._key_angle = f"Test/{self._name} Angle (deg)"

        wpilib.SmartDashboard.putNumber(self._key_speed, 0.0)
        wpilib.SmartDashboard.putNumber(self._key_angle, 0.0)

        self.addRequirements(swerve)

    def execute(self) -> None:
        """Read the dashboard values and command the module."""
        speed = wpilib.SmartDashboard.getNumber(self._key_speed, 0.0)
        angle_deg = wpilib.SmartDashboard.getNumber(self._key_angle, 0.0)

        self._module.setDesiredState(
            SwerveModuleState(speed, Rotation2d.fromDegrees(angle_deg))
        )

    def end(self, interrupted: bool) -> None:
        """Stop the chassis when the test ends."""
        self._swerve.stop()

    def isFinished(self) -> bool:
        """Run until interrupted by another command."""
        return False
