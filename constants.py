"""Hardware constants for the MK4i NEO Vortex swerve drivetrain demo.

These values are placeholders based on common MK4i configurations.
Update them to match the actual robot wiring and chosen gear ratio.
"""

import math

from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.units import inchesToMeters


class DriveConstants:
    """Physical and performance constants for the swerve drivetrain."""

    # MK4i L2 is common; change to 8.14 (L1) or 6.12 (L3) if needed.
    kDriveMotorPinionTeeth = 14
    kDriveGearRatio = 6.75  # L2 ratio
    kTurnGearRatio = 150.0 / 7.0  # MK4i steering ratio (~21.43:1)

    # 4" diameter wheels on MK4i.
    kWheelDiameterMeters = inchesToMeters(4.0)
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    # Convert motor rotations to meters (drive) and radians (turn).
    kDriveEncoderPositionFactor = kWheelCircumferenceMeters / kDriveGearRatio
    kDriveEncoderVelocityFactor = kDriveEncoderPositionFactor / 60.0

    # Chassis dimensions (distance from robot center to each module).
    # These are placeholders; measure the actual robot.
    kTrackWidth = inchesToMeters(22.5)  # left-right
    kWheelBase = inchesToMeters(22.5)   # front-back

    kModuleLocations = (
        Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),   # front left
        Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),  # front right
        Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),  # rear left
        Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0), # rear right
    )

    kDriveKinematics = SwerveDrive4Kinematics(*kModuleLocations)

    # Driver-oriented speed limits.
    kMaxSpeedMetersPerSecond = 2.8
    kMaxAngularSpeed = 2.0 * math.pi  # rad/s
    kModuleMaxAngularAcceleration = 6.0 * math.pi  # rad/s^2, for the steering profile

    # Slew rate limiters for teleop (optional smoothing).
    kDirectionSlewRate = 1.2  # rad/s
    kMagnitudeSlewRate = 1.8  # percent/s (1 = 100%)
    kRotationalSlewRate = 2.0  # percent/s (1 = 100%)

    # Deadband for driver controller input.
    kDriveDeadband = 0.09


class ModuleConstants:
    """Per-module constants (Spark MAX CAN IDs, PID gains, offsets)."""

    # Order must match DriveConstants.kModuleLocations:
    # front left, front right, rear left, rear right.
    kDriveMotorCanIds = (11, 21, 41, 31)
    kTurnMotorCanIds = (12, 22, 42, 32)

    # CTRE CANcoder CAN IDs for the azimuth absolute encoders, same order.
    kTurnCanCoderIds = (13, 23, 43, 33)

    # CANcoder rotations per full azimuth revolution (MK4i kit is 1:1).
    kTurnCanCoderGearRatio = 1.0

    # Azimuth motor direction. Matches last year's Java swerve (all modules
    # inverted) so positive voltage and positive CANcoder share a direction.
    kTurnMotorInverted = True

    # Absolute encoder zero offsets, in radians.
    # Calibrate each module so that wheel-forward gives 0 radians.
    # If your calibration tool reports rotations [0, 1), multiply by 2*pi.
    kAngularOffsets = (math.pi, 0.0, math.pi, 0.0)

    # Drive motor velocity loop gains. The loop now runs on the roboRIO and
    # outputs a voltage, so it needs a feedforward (S/V/A) in addition to PID.
    # These are starting guesses and must be tuned on the real robot.
    kDriveP = 0.1
    kDriveI = 0.01
    kDriveD = 0.0
    kDriveS = 0.3337     # volts
    kDriveV = 1.22     # volt-seconds per meter
    kDriveA = 0.13     # volt-seconds^2 per meter

    # Azimuth steering PID gains and feedforward, carried over from last
    # year's Java swerve that steered correctly.
    kTurnP = 0.95
    kTurnI = 0.0
    kTurnD = 0.25
    kTurnS = 0.8    # volts
    kTurnV = 0.41   # volt-seconds per radian
    kTurnA = 0.01   # volt-seconds^2 per radian

    # Current limits.
    kDriveCurrentLimit = 50  # amps
    kTurnCurrentLimit = 20   # amps


class VisionConstants:
    """Limelight MegaTag2 localization settings."""

    # Each entry is (NetworkTables table name, camera IP). These names are
    # the "NetworkTables Name" values currently configured on the cameras.
    # The field map and the camera's robot-space pose (LL Forward/Right/Up/
    # Roll/Pitch/Yaw) must also be configured in each camera's web UI.
    kLimelights = (
        ("limelight-left", "10.83.26.17"),
        ("limelight-front", "10.83.26.18"),
        ("limelight-right", "10.83.26.19"),
    )

    # The yaw sent to MegaTag2 is CCW-positive and 0 degrees must point at
    # the red alliance wall, so zero the NavX in that orientation at startup.
    # MegaTag2 estimates are not trusted while the robot spins faster than
    # this. NavX getRate() is in degrees per second.
    kMaxAngularVelocityDps = 360.0

    # Docs recommendation: trust vision translation, but not its heading.
    # MegaTag2 heading comes from the gyro sent in robot_orientation_set.
    kVisionStdDevs = (0.7, 0.7, 9999999.0)


class OIConstants:
    """Operator interface constants."""

    kDriverControllerPort = 0
