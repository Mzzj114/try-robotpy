import commands2

from constants import DriveConstants


class TurtleMode(commands2.Command):
    def __init__(self) -> None:
        super().__init__()

    def initialize(self) -> None:
        print("turtle mode activated")
        DriveConstants.kMaxSpeedMetersPerSecond = (
            DriveConstants.kMaxSpeedMetersPerSecondStorage
        )
        DriveConstants.kMaxSpeedMetersPerSecond /= 7

    def end(self, interrupted: bool) -> None:
        DriveConstants.kMaxSpeedMetersPerSecond = (
            DriveConstants.kMaxSpeedMetersPerSecondStorage
        )
