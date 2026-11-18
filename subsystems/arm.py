from dataclasses import dataclass

import commands2
import wpilib
from pathplannerlib.config import PIDConstants, RobotConfig
from wpilib.sysid import SysIdRoutineLog
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModulePosition,
    SwerveModuleState,
)

from constants import (
    AutoConstants,
    DriveConstants,
    ModuleConstants,
    SteeringSysIdConstants,
    VisionConstants,
)
import rev
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    SimpleMotorFeedforwardMeters,
    ArmFeedforward,
)
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.trajectory import TrapezoidProfileRadians

from constants import DriveConstants, ModuleConstants

class Arm (commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self._drive_motor = rev.SparkMax(
            drive_can_id, rev.SparkLowLevel.MotorType.kBrushless
        )
