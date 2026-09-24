"""Swerve drivetrain subsystem for an MK4i NEO Vortex chassis.

Uses four SwerveModule instances, wpimath SwerveDrive4Kinematics /
SwerveDrive4Odometry, and a NavX gyro for field-oriented control.

References:
- WPILib swerve odometry: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
- NavX Python API: https://robotpy.readthedocs.io/projects/navx/en/stable/api.html
"""

import commands2
import navx
import wpilib
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)

from constants import DriveConstants, ModuleConstants
from subsystems.swerve_module import SwerveModule


class SwerveDrive(commands2.Subsystem):
    """Coordinates four swerve modules and tracks field pose."""

    # Order matches DriveConstants.kModuleLocations.
    _module_names = ("Front Left", "Front Right", "Rear Left", "Rear Right")

    def __init__(self) -> None:
        super().__init__()

        self._gyro = navx.AHRS(
            navx.AHRS.NavXComType.kMXP_SPI,
            navx.AHRS.NavXUpdateRate.k100Hz,
        )

        self._modules: tuple[
            SwerveModule, SwerveModule, SwerveModule, SwerveModule
        ] = (
            SwerveModule(
                ModuleConstants.kDriveMotorCanIds[0],
                ModuleConstants.kTurnMotorCanIds[0],
                ModuleConstants.kTurnCanCoderIds[0],
                ModuleConstants.kAngularOffsets[0],
            ),
            SwerveModule(
                ModuleConstants.kDriveMotorCanIds[1],
                ModuleConstants.kTurnMotorCanIds[1],
                ModuleConstants.kTurnCanCoderIds[1],
                ModuleConstants.kAngularOffsets[1],
            ),
            SwerveModule(
                ModuleConstants.kDriveMotorCanIds[2],
                ModuleConstants.kTurnMotorCanIds[2],
                ModuleConstants.kTurnCanCoderIds[2],
                ModuleConstants.kAngularOffsets[2],
            ),
            SwerveModule(
                ModuleConstants.kDriveMotorCanIds[3],
                ModuleConstants.kTurnMotorCanIds[3],
                ModuleConstants.kTurnCanCoderIds[3],
                ModuleConstants.kAngularOffsets[3],
            ),
        )

        self._odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            self.getRotation2d(),
            self._modulePositions(),
        )

        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

        self._desired_chassis_speeds = ChassisSpeeds()

    def periodic(self) -> None:
        """Update odometry and publish telemetry every scheduler loop."""
        self._odometry.update(
            self.getRotation2d(),
            self._modulePositions(),
        )

        pose = self.getPose()
        self._field.setRobotPose(pose)

        wpilib.SmartDashboard.putNumber("Swerve/Pose X", pose.x)
        wpilib.SmartDashboard.putNumber("Swerve/Pose Y", pose.y)
        wpilib.SmartDashboard.putNumber(
            "Swerve/Pose Theta", pose.rotation().degrees()
        )

        # Battery voltage confirms the bus is healthy while testing motors.
        wpilib.SmartDashboard.putNumber(
            "Diagnostics/Battery Voltage (V)",
            wpilib.RobotController.getBatteryVoltage(),
        )

        # Per-module telemetry. Compare "Angle" against "Angle Desired" to
        # verify the azimuth motors are actually steering the modules.
        for name, module in zip(self._module_names, self._modules):
            module.publishTelemetry(f"Swerve/{name}")

    def getPose(self) -> Pose2d:
        """Return the current field-relative pose."""
        return self._odometry.getPose()

    def resetPose(self, pose: Pose2d) -> None:
        """Reset the odometry to a known pose."""
        self._odometry.resetPosition(
            self.getRotation2d(),
            self._modulePositions(),
            pose,
        )

    def _modulePositions(
        self,
    ) -> tuple[
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
        SwerveModulePosition,
    ]:
        """Return the four module positions as a fixed-size tuple.

        WPILib's SwerveDrive4Odometry signatures require exactly four
        positions, so the unpacking guarantees a concrete 4-tuple type.
        """
        front_left, front_right, rear_left, rear_right = self._modules
        return (
            front_left.getPosition(),
            front_right.getPosition(),
            rear_left.getPosition(),
            rear_right.getPosition(),
        )

    def resetEncoders(self) -> None:
        """Zero all drive wheel encoders."""
        for module in self._modules:
            module.resetEncoders()

    def zeroHeading(self) -> None:
        """Zero the gyro yaw; odometry translation is preserved."""
        self._gyro.zeroYaw()

    def getRotation2d(self) -> Rotation2d:
        """Return the robot heading as a CCW-positive Rotation2d."""
        return Rotation2d.fromDegrees(-self._gyro.getAngle())

    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rot: float,
        field_relative: bool,
    ) -> None:
        """Drive the robot with controller-style inputs.

        :param x_speed: Forward velocity in m/s (positive = forward).
        :param y_speed: Leftward velocity in m/s (positive = left).
        :param rot: Counterclockwise angular velocity in rad/s.
        :param field_relative: True to use field-oriented control.
        """
        if field_relative:
            chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot, self.getRotation2d()
            )
        else:
            chassis_speeds = ChassisSpeeds(x_speed, y_speed, rot)

        self.driveRobotRelative(chassis_speeds)

    def driveRobotRelative(self, chassis_speeds: ChassisSpeeds) -> None:
        """Command the modules directly from a ChassisSpeeds object."""
        self._desired_chassis_speeds = chassis_speeds

        wpilib.SmartDashboard.putNumber("Swerve/Chassis Speeds/vx (m/s)", chassis_speeds.vx)
        wpilib.SmartDashboard.putNumber("Swerve/Chassis Speeds/vy (m/s)", chassis_speeds.vy)
        wpilib.SmartDashboard.putNumber("Swerve/Chassis Speeds/omega (deg/s)", chassis_speeds.omega_dps)

        module_states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            chassis_speeds
        )
        module_states = DriveConstants.kDriveKinematics.desaturateWheelSpeeds(
            module_states, DriveConstants.kMaxSpeedMetersPerSecond
        )

        for module, state in zip(self._modules, module_states):
            module.setDesiredState(state)

    def stop(self) -> None:
        """Stop all modules and hold their current azimuth."""
        self._desired_chassis_speeds = ChassisSpeeds()
        for module in self._modules:
            module.stop()

    def getDesiredChassisSpeeds(self) -> ChassisSpeeds:
        """Return the most recent chassis speeds commanded by teleop/auto."""

        chassis_speeds = self._desired_chassis_speeds
        
        return chassis_speeds

    def setX(self) -> None:
        """Park the modules in an X orientation to resist pushing."""
        self._desired_chassis_speeds = ChassisSpeeds()
        x_angles = (
            Rotation2d.fromDegrees(45.0),
            Rotation2d.fromDegrees(-45.0),
            Rotation2d.fromDegrees(135.0),
            Rotation2d.fromDegrees(-135.0),
        )
        for module, angle in zip(self._modules, x_angles):
            module.setDesiredState(SwerveModuleState(0.0, angle))

    def getModules(self) -> tuple:
        """Return the four swerve modules for simulation access."""
        return self._modules

    def getModuleNames(self) -> tuple:
        """Return the module display names, matching kModuleLocations order."""
        return self._module_names

    def getGyro(self) -> navx.AHRS:
        """Return the NavX gyro for simulation access."""
        return self._gyro

    def getModuleStates(self) -> tuple:
        """Return the current state of each module for telemetry."""
        return tuple(module.getState() for module in self._modules)
