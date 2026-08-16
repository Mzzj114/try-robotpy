"""Arcade drive command using a joystick supplier."""

import typing

import commands2

from subsystems.drivetrain import Drivetrain


class ArcadeDrive(commands2.Command):
    """Default teleop command: drives the robot with arcade controls."""

    def __init__(
        self,
        drivetrain: Drivetrain,
        forward_supplier: typing.Callable[[], float],
        rotation_supplier: typing.Callable[[], float],
    ) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.forward_supplier = forward_supplier
        self.rotation_supplier = rotation_supplier
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        self.drivetrain.arcadeDrive(
            self.forward_supplier(), self.rotation_supplier()
        )

    def end(self, interrupted: bool) -> None:
        self.drivetrain.stop()

    def isFinished(self) -> bool:
        return False
