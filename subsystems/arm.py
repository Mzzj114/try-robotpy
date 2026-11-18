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
    ArmConstants,
    ArmConstants,
    SteeringSysIdConstants,
    VisionConstants,
    ArmConstants
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
import math

from constants import ArmConstants, ArmConstants

class Arm (commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self._arm_motors=[] 

        for canID in ArmConstants.kFirstStageMotorIDs:
            self._arm_motor.append(rev.SparkMax(canID, rev.SparkLowLevel.MotorType.kBrushless))
        
            #configuration
            arm_config = rev.SparkMaxConfig()
            arm_config.apply(rev.SparkBaseConfig.Presets.REV_Vortex())
            arm_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            arm_config.smartCurrentLimit(ArmConstants.kDriveCurrentLimit)
            arm_config.encoder.positionConversionFactor(
                ArmConstants.kAngleConversionFactor
            )
            arm_config.encoder.velocityConversionFactor(
                ArmConstants.kAngleConversionFactor
            )
            self.arm_motors[-1].configure(
                arm_config,
                rev.ResetMode.kResetSafeParameters,
                rev.PersistMode.kNoPersistParameters,)
            
        self._first_stage_pid = ProfiledPIDControllerRadians(
            ArmConstants.kFirstStageArmP,
            ArmConstants.kFirstStageArmI,
            ArmConstants.kFirstStageArmD,
            TrapezoidProfileRadians.Constraints(
                ArmConstants.kMaxAngularSpeed,
                ArmConstants.kModuleMaxAngularAcceleration,
            ))
        self._first_stage_arm_pid.enableContinuousInput(-math.pi, math.pi)
        self._first_stage_arm_feedforward = ArmFeedforward(
                    ArmConstants.kFirstStageArmS,
                    ArmConstants.kFirstStageArmG,
                    ArmConstants.kFirstStageArmV,
                )
    def setDesiredAngle(self, angle: float):
        '''
        input angle radians
        
        zero when arm is parallel to ground and facing front of the robot
        '''
