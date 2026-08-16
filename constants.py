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
    kTurnEncoderPositionFactor = (2.0 * math.pi) / kTurnGearRatio
    kTurnEncoderVelocityFactor = kTurnEncoderPositionFactor / 60.0

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
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = 2.0 * math.pi  # rad/s

    # Slew rate limiters for teleop (optional smoothing).
    kDirectionSlewRate = 1.2  # rad/s
    kMagnitudeSlewRate = 1.8  # percent/s (1 = 100%)
    kRotationalSlewRate = 2.0  # percent/s (1 = 100%)

    # Deadband for joystick input.
    kDriveDeadband = 0.05


class ModuleConstants:
    """Per-module constants (Spark MAX CAN IDs, PID gains, offsets)."""

    # Order must match DriveConstants.kModuleLocations:
    # front left, front right, rear left, rear right.
    kDriveMotorCanIds = (1, 3, 5, 7)
    kTurnMotorCanIds = (2, 4, 6, 8)

    # Absolute encoder zero offsets, in radians.
    # Calibrate each module so that wheel-forward gives 0 radians.
    # If your calibration tool reports rotations [0, 1), multiply by 2*pi.
    kAngularOffsets = (0.0, 0.0, 0.0, 0.0)

    # Drive motor closed-loop gains (NEO Vortex velocity mode).
    # These are starting guesses and must be tuned on the real robot.
    kDriveP = 0.1
    kDriveI = 0.0
    kDriveD = 0.0
    kDriveFF = 0.0

    # Turn motor closed-loop gains (position mode, radians).
    kTurnP = 1.0
    kTurnI = 0.0
    kTurnD = 0.0

    # Current limits.
    kDriveCurrentLimit = 60  # amps
    kTurnCurrentLimit = 20   # amps


class OIConstants:
    """Operator interface constants."""

    kDriverControllerPort = 0
