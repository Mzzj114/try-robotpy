"""A single MK4i swerve module using two Spark MAX controllers.

The drive motor is a NEO Vortex (velocity closed-loop on the built-in encoder).
The azimuth motor is a NEO 550 (position closed-loop on the absolute encoder).

References:
- REV Spark MAX API: https://robotpy.readthedocs.io/projects/rev/en/stable/api.html
- WPILib swerve kinematics: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
"""

import math

import rev
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from constants import DriveConstants, ModuleConstants


class SwerveModule:
    """Controls one swerve module: a drive motor and a steering motor."""

    def __init__(self, drive_can_id: int, turn_can_id: int, angular_offset: float) -> None:
        """Construct a swerve module.

        :param drive_can_id: CAN ID for the NEO Vortex drive Spark MAX.
        :param turn_can_id: CAN ID for the NEO 550 azimuth Spark MAX.
        :param angular_offset: Radians to subtract from the absolute encoder
            so that a wheel angle of 0 corresponds to robot forward.
        """
        self._angular_offset = Rotation2d(angular_offset)

        # ---- Drive motor (NEO Vortex) ----
        self._drive_motor = rev.SparkMax(
            drive_can_id, rev.SparkLowLevel.MotorType.kBrushless
        )

        drive_config = rev.SparkMaxConfig()
        drive_config.apply(rev.SparkBaseConfig.Presets.REV_Vortex())
        drive_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        drive_config.smartCurrentLimit(ModuleConstants.kDriveCurrentLimit)
        drive_config.encoder.positionConversionFactor(
            DriveConstants.kDriveEncoderPositionFactor
        )
        drive_config.encoder.velocityConversionFactor(
            DriveConstants.kDriveEncoderVelocityFactor
        )
        drive_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        ).pid(
            ModuleConstants.kDriveP,
            ModuleConstants.kDriveI,
            ModuleConstants.kDriveD,
        )

        self._drive_motor.configure(
            drive_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

        # ---- Azimuth motor (NEO 550) ----
        self._turn_motor = rev.SparkMax(
            turn_can_id, rev.SparkLowLevel.MotorType.kBrushless
        )

        turn_config = rev.SparkMaxConfig()
        turn_config.apply(rev.SparkBaseConfig.Presets.REV_NEO_550())
        turn_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        turn_config.smartCurrentLimit(ModuleConstants.kTurnCurrentLimit)
        turn_config.absoluteEncoder.positionConversionFactor(
            DriveConstants.kTurnEncoderPositionFactor
        )
        turn_config.absoluteEncoder.velocityConversionFactor(
            DriveConstants.kTurnEncoderVelocityFactor
        )
        turn_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kAbsoluteEncoder
        ).pid(
            ModuleConstants.kTurnP,
            ModuleConstants.kTurnI,
            ModuleConstants.kTurnD,
        )
        turn_config.closedLoop.positionWrappingEnabled(True)
        turn_config.closedLoop.positionWrappingInputRange(-math.pi, math.pi)

        self._turn_motor.configure(
            turn_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

        # Convenience handles for sensors and controllers.
        self._drive_encoder = self._drive_motor.getEncoder()
        self._turn_encoder = self._turn_motor.getAbsoluteEncoder()
        self._drive_pid = self._drive_motor.getClosedLoopController()
        self._turn_pid = self._turn_motor.getClosedLoopController()

        self._desired_state = SwerveModuleState(0.0, Rotation2d())

    def getState(self) -> SwerveModuleState:
        """Return the current module state (wheel speed and angle)."""
        return SwerveModuleState(
            self._drive_encoder.getVelocity(),
            Rotation2d(self._turn_encoder.getPosition()) - self._angular_offset,
        )

    def getPosition(self) -> SwerveModulePosition:
        """Return the current module position (distance and angle)."""
        return SwerveModulePosition(
            self._drive_encoder.getPosition(),
            Rotation2d(self._turn_encoder.getPosition()) - self._angular_offset,
        )

    def setDesiredState(self, desired_state: SwerveModuleState) -> None:
        """Command the module to a speed and angle.

        Applies SwerveModuleState.optimize to minimize wheel rotation and
        cosine compensation to reduce lateral force while turning.
        """
        current_rotation = Rotation2d(self._turn_encoder.getPosition())

        # Target encoder angle = desired wheel angle + mechanical offset.
        target = SwerveModuleState(
            desired_state.speed,
            desired_state.angle + self._angular_offset,
        )
        target.optimize(current_rotation)
        target.cosineScale(current_rotation)

        self._drive_pid.setSetpoint(
            target.speed, rev.SparkLowLevel.ControlType.kVelocity
        )
        self._turn_pid.setSetpoint(
            target.angle.radians(), rev.SparkLowLevel.ControlType.kPosition
        )

        self._desired_state = desired_state

    def get_drive_motor(self) -> rev.SparkMax:
        """Return the drive Spark MAX for simulation access."""
        return self._drive_motor

    def get_turn_motor(self) -> rev.SparkMax:
        """Return the azimuth Spark MAX for simulation access."""
        return self._turn_motor

    def stop(self) -> None:
        """Set drive output to zero and hold the current wheel angle."""
        self._drive_motor.set(0.0)
        self._turn_pid.setSetpoint(
            self._turn_encoder.getPosition(),
            rev.SparkLowLevel.ControlType.kPosition,
        )

    def resetEncoders(self) -> None:
        """Zero the drive encoder. The absolute turn encoder cannot be reset."""
        self._drive_encoder.setPosition(0.0)
