"""A single MK4i swerve module using two Spark MAX controllers.

Both control loops run on the roboRIO and output a voltage to their Spark MAX.
The drive motor is a NEO Vortex whose velocity loop reads the Spark MAX
built-in encoder. The azimuth motor is a NEO 550 whose position loop reads a
CTRE CANcoder on the steering axis. A CANcoder talks over CAN, so it cannot
feed the Spark MAX data port; the position loop must live on the roboRIO
instead of on the Spark MAX.

References:
- REV Spark MAX API: https://robotpy.readthedocs.io/projects/rev/en/stable/api.html
- CTRE CANcoder: https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/cancoder/index.html
- WPILib swerve kinematics: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
"""

import math

import rev
import wpilib
from phoenix6.configs import CANcoderConfiguration, MagnetSensorConfigs
from phoenix6.hardware import CANcoder
from phoenix6.signals import SensorDirectionValue
from wpimath.controller import (
    PIDController,
    ProfiledPIDControllerRadians,
    SimpleMotorFeedforwardMeters,
    SimpleMotorFeedforwardRadians,
)
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.trajectory import TrapezoidProfileRadians

from constants import DriveConstants, ModuleConstants


class SwerveModule:
    """Controls one swerve module: a drive motor and a steering motor."""

    def __init__(
        self,
        drive_can_id: int,
        turn_can_id: int,
        can_coder_id: int,
        angular_offset: float,
    ) -> None:
        """Construct a swerve module.

        :param drive_can_id: CAN ID for the NEO Vortex drive Spark MAX.
        :param turn_can_id: CAN ID for the NEO 550 azimuth Spark MAX.
        :param can_coder_id: CAN ID for the azimuth CTRE CANcoder.
        :param angular_offset: Radians to subtract from the CANcoder reading so
            that a wheel angle of 0 corresponds to robot forward.
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

        self._drive_motor.configure(
            drive_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

        # ---- Azimuth motor (NEO 550), voltage control only ----
        self._turn_motor = rev.SparkMax(
            turn_can_id, rev.SparkLowLevel.MotorType.kBrushless
        )

        turn_config = rev.SparkMaxConfig()
        turn_config.apply(rev.SparkBaseConfig.Presets.REV_NEO_550())
        turn_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        turn_config.smartCurrentLimit(ModuleConstants.kTurnCurrentLimit)
        turn_config.inverted(ModuleConstants.kTurnMotorInverted)

        self._turn_motor.configure(
            turn_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

        # ---- Azimuth absolute encoder (CANcoder) ----
        self._can_coder = CANcoder(can_coder_id)
        self._can_coder.configurator.apply(
            CANcoderConfiguration().with_magnet_sensor(
                MagnetSensorConfigs()
                .with_sensor_direction(
                    SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
                )
                .with_magnet_offset(0.0)
            )
        )
        self._turn_position_signal = self._can_coder.get_absolute_position()
        self._magnet_health_signal = self._can_coder.get_magnet_health()

        # Steering profiled PID + feedforward runs on the roboRIO because the
        # Spark MAX has no encoder. Continuous input lets the wheel take the
        # shortest path across +/- pi; the profile limits angular rate.
        self._turn_pid = ProfiledPIDControllerRadians(
            ModuleConstants.kTurnP,
            ModuleConstants.kTurnI,
            ModuleConstants.kTurnD,
            TrapezoidProfileRadians.Constraints(
                DriveConstants.kMaxAngularSpeed,
                DriveConstants.kModuleMaxAngularAcceleration,
            ),
        )
        self._turn_pid.enableContinuousInput(-math.pi, math.pi)
        self._turn_feedforward = SimpleMotorFeedforwardRadians(
            ModuleConstants.kTurnS,
            ModuleConstants.kTurnV,
            ModuleConstants.kTurnA,
        )

        # Drive velocity PID + feedforward also runs on the roboRIO. The Spark
        # MAX built-in encoder is the feedback, but the loop lives here so both
        # swerve loops can be tuned and logged the same way.
        self._drive_pid = PIDController(
            ModuleConstants.kDriveP,
            ModuleConstants.kDriveI,
            ModuleConstants.kDriveD,
        )
        self._drive_feedforward = SimpleMotorFeedforwardMeters(
            ModuleConstants.kDriveS,
            ModuleConstants.kDriveV,
            ModuleConstants.kDriveA,
        )

        # Convenience handle for the drive encoder.
        self._drive_encoder = self._drive_motor.getEncoder()

        # Cache once: firmware/serial are blocking CAN reads, not loop-safe.
        self._drive_firmware = self._drive_motor.getFirmwareString()
        self._turn_firmware = self._turn_motor.getFirmwareString()
        self._drive_serial = self._drive_motor.getSerialNumber()
        self._turn_serial = self._turn_motor.getSerialNumber()

        self._desired_state = SwerveModuleState(0.0, Rotation2d())
        self._turn_target = Rotation2d()
        self._turn_voltage_command = 0.0
        self._drive_voltage_command = 0.0

    def _get_turn_radians(self) -> float:
        """Read the CANcoder azimuth in radians, including the offset."""
        self._turn_position_signal.refresh()
        rotations = self._turn_position_signal.value
        return (
            rotations * (2.0 * math.pi) / ModuleConstants.kTurnCanCoderGearRatio
        )

    def getState(self) -> SwerveModuleState:
        """Return the current module state (wheel speed and angle)."""
        return SwerveModuleState(
            self._drive_encoder.getVelocity(),
            Rotation2d(self._get_turn_radians()) - self._angular_offset,
        )

    def getPosition(self) -> SwerveModulePosition:
        """Return the current module position (distance and angle)."""
        return SwerveModulePosition(
            self._drive_encoder.getPosition(),
            Rotation2d(self._get_turn_radians()) - self._angular_offset,
        )

    def getDesiredState(self) -> SwerveModuleState:
        """Return the most recent wheel-frame state commanded to this module."""
        return self._desired_state

    def publishTelemetry(self, prefix: str) -> None:
        """Publish angles, speeds, and electrical data for this module.

        :param prefix: SmartDashboard key prefix, e.g. ``Swerve/Front Left``.
        """
        # Read the CANcoder once per publish to avoid redundant CAN traffic.
        turn_radians = self._get_turn_radians()
        turn_rotation = Rotation2d(turn_radians)
        desired = self._turn_target - self._angular_offset

        # Shortest-path control error, so the value stays in [-180, 180] deg
        # even though the wheels may take the long way via optimize().
        error_deg = (self._turn_target - turn_rotation).degrees()

        # Steer angle tracking: if Angle Error never shrinks, the azimuth
        # motor is not reaching its setpoint (wiring, encoder, or PID issue).
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Angle (deg)", (turn_rotation - self._angular_offset).degrees()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Angle Desired (deg)", desired.degrees()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Angle Error (deg)", error_deg
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Speed (m/s)", self._drive_encoder.getVelocity()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Speed Desired (m/s)", self._desired_state.speed
        )

        # CANcoder reading in radians. Constant while steering means the
        # encoder is not moving or is not on the bus.
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Turn Encoder (rad)", turn_radians
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Turn Output", self._turn_motor.getAppliedOutput()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Turn Command (V)", self._turn_voltage_command
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Turn Current (A)", self._turn_motor.getOutputCurrent()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Turn Bus Voltage (V)", self._turn_motor.getBusVoltage()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Drive Output", self._drive_motor.getAppliedOutput()
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Drive Command (V)", self._drive_voltage_command
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Drive Current (A)", self._drive_motor.getOutputCurrent()
        )

        # Identity and faults. Empty firmware or serial 0 means the controller
        # is not on the CAN bus.
        wpilib.SmartDashboard.putString(
            f"{prefix}/Turn Firmware", self._turn_firmware
        )
        wpilib.SmartDashboard.putString(
            f"{prefix}/Drive Firmware", self._drive_firmware
        )
        wpilib.SmartDashboard.putString(
            f"{prefix}/Turn Serial", str(self._turn_serial)
        )
        wpilib.SmartDashboard.putString(
            f"{prefix}/Drive Serial", str(self._drive_serial)
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Turn Fault Bits", float(self._turn_motor.getFaults().rawBits)
        )
        wpilib.SmartDashboard.putNumber(
            f"{prefix}/Turn Sticky Fault Bits",
            float(self._turn_motor.getStickyFaults().rawBits),
        )

        # A CANcoder with a bad magnet or dropped bus breaks steering.
        self._magnet_health_signal.refresh()
        wpilib.SmartDashboard.putBoolean(
            f"{prefix}/CANcoder Connected", self._can_coder.is_connected
        )
        wpilib.SmartDashboard.putString(
            f"{prefix}/CANcoder Magnet Health",
            str(self._magnet_health_signal.value),
        )

    def setDesiredState(self, desired_state: SwerveModuleState) -> None:
        """Command the module to a speed and angle.

        Applies SwerveModuleState.optimize to minimize wheel rotation and
        cosine compensation to reduce lateral force while turning.
        """
        current_rotation = Rotation2d(self._get_turn_radians())

        # Target encoder angle = desired wheel angle + mechanical offset.
        target = SwerveModuleState(
            desired_state.speed,
            desired_state.angle + self._angular_offset,
        )
        target.optimize(current_rotation)
        target.cosineScale(current_rotation)

        # Drive velocity loop on the roboRIO: feedforward on the target speed
        # plus PID on the measured speed, output as a voltage.
        measured_speed = self._drive_encoder.getVelocity()
        self._drive_voltage_command = self._drive_feedforward.calculate(
            target.speed
        ) + self._drive_pid.calculate(measured_speed, target.speed)
        self._drive_motor.setVoltage(self._drive_voltage_command)

        # The azimuth motor is voltage-driven; the roboRIO closes the loop on
        # the CANcoder and adds a profile-based feedforward.
        turn_output = self._turn_pid.calculate(
            current_rotation.radians(), target.angle.radians()
        )
        turn_feedforward = self._turn_feedforward.calculate(
            self._turn_pid.getSetpoint().velocity
        )
        self._turn_voltage_command = turn_output + turn_feedforward
        self._turn_motor.setVoltage(turn_output + turn_feedforward)

        self._turn_target = target.angle
        self._desired_state = desired_state

    def get_drive_motor(self) -> rev.SparkMax:
        """Return the drive Spark MAX for simulation access."""
        return self._drive_motor

    def get_turn_motor(self) -> rev.SparkMax:
        """Return the azimuth Spark MAX for simulation access."""
        return self._turn_motor

    def get_drive_pid_controller(self) -> PIDController:
        """Return the roboRIO drive velocity PID for live tuning."""
        return self._drive_pid

    def get_turn_pid_controller(self) -> ProfiledPIDControllerRadians:
        """Return the roboRIO steering PID for live tuning."""
        return self._turn_pid

    def get_can_coder(self) -> CANcoder:
        """Return the azimuth CANcoder for simulation access."""
        return self._can_coder

    def stop(self) -> None:
        """Stop drive, hold the azimuth angle, and reset both control loops."""
        self._drive_motor.setVoltage(0.0)
        # self._drive_pid.reset()
        # self._turn_pid.reset(self._get_turn_radians())
        self._turn_motor.setVoltage(0.0)

    def resetEncoders(self) -> None:
        """Zero the drive encoder. The absolute turn encoder cannot be reset."""
        self._drive_encoder.setPosition(0.0)
