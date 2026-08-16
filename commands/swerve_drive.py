"""Default teleop command for the swerve drivetrain.

Maps joystick inputs to field-oriented or robot-oriented ChassisSpeeds.
"""

import math
from typing import Callable

import commands2

from constants import DriveConstants
from subsystems.swerve_drive import SwerveDrive


class SwerveDriveCommand(commands2.Command):
    """Drive the swerve chassis from joystick suppliers."""

    def __init__(
        self,
        swerve: SwerveDrive,
        x_supplier: Callable[[], float],
        y_supplier: Callable[[], float],
        rot_supplier: Callable[[], float],
        field_relative: bool = True,
    ) -> None:
        """Construct the command.

        :param swerve: The swerve drivetrain subsystem.
        :param x_supplier: Forward axis supplier, range [-1, 1].
        :param y_supplier: Leftward axis supplier, range [-1, 1].
        :param rot_supplier: Counterclockwise rotation axis, range [-1, 1].
        :param field_relative: True for field-oriented control.
        """
        super().__init__()
        self._swerve = swerve
        self._x_supplier = x_supplier
        self._y_supplier = y_supplier
        self._rot_supplier = rot_supplier
        self._field_relative = field_relative

        self.addRequirements(swerve)

    def execute(self) -> None:
        """Read inputs, apply deadband and scaling, and command the chassis."""
        x = self._apply_deadband(self._x_supplier())
        y = self._apply_deadband(self._y_supplier())
        rot = self._apply_deadband(self._rot_supplier())

        # Square the inputs for finer low-speed control.
        x = math.copysign(x * x, x)
        y = math.copysign(y * y, y)
        rot = math.copysign(rot * rot, rot)

        self._swerve.drive(
            x * DriveConstants.kMaxSpeedMetersPerSecond,
            y * DriveConstants.kMaxSpeedMetersPerSecond,
            rot * DriveConstants.kMaxAngularSpeed,
            self._field_relative,
        )

    def end(self, interrupted: bool) -> None:
        """Stop the chassis when the command ends."""
        self._swerve.stop()

    def isFinished(self) -> bool:
        """Run until interrupted by another command."""
        return False

    def toggle_field_relative(self) -> None:
        """Switch between field-oriented and robot-oriented drive."""
        self._field_relative = not self._field_relative

    @staticmethod
    def _apply_deadband(value: float) -> float:
        """Zero small joystick inputs to avoid drift."""
        if abs(value) < DriveConstants.kDriveDeadband:
            return 0.0
        # Rescale so output is continuous after the deadband.
        return (value - math.copysign(DriveConstants.kDriveDeadband, value)) / (
            1.0 - DriveConstants.kDriveDeadband
        )
