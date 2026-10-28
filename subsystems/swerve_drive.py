"""Swerve drivetrain subsystem for an MK4i NEO Vortex chassis.

Uses four SwerveModule instances, wpimath SwerveDrive4Kinematics /
SwerveDrive4PoseEstimator, a NavX gyro for field-oriented control, and three
Limelight cameras providing MegaTag2 vision measurements.

References:
- WPILib swerve pose estimation: https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-pose-estimator.html
- Limelight MegaTag2: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
- NavX Python API: https://robotpy.readthedocs.io/projects/navx/en/stable/api.html
"""

import commands2
import navx
import wpilib
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from wpilib.sysid import SysIdRoutineLog
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModulePosition,
    SwerveModuleState,
)

from constants import (
    AutoConstants,
    DriveConstants,
    ModuleConstants,
    SysIdConstants,
    VisionConstants,
)
from subsystems.limelight import Limelight
from subsystems.swerve_module import SwerveModule


class SwerveDrive(commands2.Subsystem):
    """Coordinates four swerve modules and tracks field pose."""

    # Order matches DriveConstants.kModuleLocations.
    _module_names = ("Front Left", "Front Right", "Rear Left", "Rear Right")
    _sysid_module_names = (
        "front-left",
        "front-right",
        "rear-left",
        "rear-right",
    )

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

        self._pose_estimator = SwerveDrive4PoseEstimator(
            DriveConstants.kDriveKinematics,
            self.getRotation2d(),
            self._modulePositions(),
            Pose2d(),
        )

        # One NetworkTables client per Limelight. Each camera must have a
        # unique NetworkTables name matching VisionConstants.kLimelights.
        self._limelights = tuple(
            Limelight(table_name, ip)
            for table_name, ip in VisionConstants.kLimelights
        )
        self._last_vision_source_us = [0] * len(self._limelights)
        self._vision_accepted_counts = [0] * len(self._limelights)

        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

        self._desired_chassis_speeds = ChassisSpeeds()

        # PathPlanner integration. RobotConfig reads the GUI project settings
        # in deploy/pathplanner/settings.json (mass, module positions, wheel
        # radius, gearing), so the GUI must stay the source of truth for the
        # robot geometry.
        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, _feedforwards: self.driveRobotRelative(speeds),
            PPHolonomicDriveController(
                PIDConstants(
                    AutoConstants.kTranslationP,
                    AutoConstants.kTranslationI,
                    AutoConstants.kTranslationD,
                ),
                PIDConstants(
                    AutoConstants.kRotationP,
                    AutoConstants.kRotationI,
                    AutoConstants.kRotationD,
                ),
            ),
            RobotConfig.fromGUISettings(),
            self._should_flip_path,
            self,
        )

    @staticmethod
    def _should_flip_path() -> bool:
        """Mirror paths for the red alliance while keeping a blue origin."""
        return (
            wpilib.DriverStation.getAlliance()
            == wpilib.DriverStation.Alliance.kRed
        )

    def periodic(self) -> None:
        """Update the pose estimate and publish telemetry every loop."""
        self._pose_estimator.update(
            self.getRotation2d(),
            self._modulePositions(),
        )
        self._update_vision()

        pose = self.getPose()
        self._field.setRobotPose(pose)

        # Estimator result (odometry fused with MegaTag2 measurements).
        wpilib.SmartDashboard.putNumber(
            "Vision/Estimated Pose X (m)", pose.x
        )
        wpilib.SmartDashboard.putNumber(
            "Vision/Estimated Pose Y (m)", pose.y
        )
        wpilib.SmartDashboard.putNumber(
            "Vision/Estimated Pose Theta (deg)", pose.rotation().degrees()
        )

        # Legacy keys kept for existing dashboards.
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

    def _update_vision(self) -> None:
        """Send the gyro heading to each camera and fuse new MegaTag2 frames."""
        yaw_degrees = self.getRotation2d().degrees()
        spinning = (
            abs(self._gyro.getRate())
            > VisionConstants.kMaxAngularVelocityDps
        )

        for index, camera in enumerate(self._limelights):
            # MegaTag2 needs the current yaw before it computes a pose. The
            # angular rates are optional, so they stay at zero (as in the
            # Limelight documentation example).
            camera.set_robot_orientation(yaw_degrees)

            _heartbeat, connected = camera.update_status()
            estimate = camera.get_megatag2_estimate()

            tag_count = estimate.tag_count if estimate is not None else 0
            accepted = False
            status = "NO CAMERA"

            if connected:
                if estimate is None:
                    status = "NO TAGS"
                elif estimate.source_time_us == self._last_vision_source_us[index]:
                    status = "SAME FRAME"
                else:
                    # Mark the frame as seen even when it is rejected, so a
                    # stale frame is never added later.
                    self._last_vision_source_us[index] = estimate.source_time_us
                    if spinning:
                        status = "REJECTED: SPINNING"
                    else:
                        self._pose_estimator.addVisionMeasurement(
                            estimate.pose,
                            estimate.timestamp_seconds,
                            VisionConstants.kVisionStdDevs,
                        )
                        self._vision_accepted_counts[index] += 1
                        accepted = True
                        status = "ACCEPTED"

            prefix = f"Vision/{camera.table_name}"
            wpilib.SmartDashboard.putBoolean(f"{prefix}/Connected", connected)
            wpilib.SmartDashboard.putBoolean(
                f"{prefix}/Has Target", camera.has_target()
            )
            wpilib.SmartDashboard.putNumber(f"{prefix}/Tag Count", tag_count)
            wpilib.SmartDashboard.putBoolean(
                f"{prefix}/Frame Accepted", accepted
            )
            wpilib.SmartDashboard.putNumber(
                f"{prefix}/Accepted Measurements",
                self._vision_accepted_counts[index],
            )
            wpilib.SmartDashboard.putString(f"{prefix}/Status", status)

            if estimate is not None:
                wpilib.SmartDashboard.putNumber(
                    f"{prefix}/Pose X (m)", estimate.pose.x
                )
                wpilib.SmartDashboard.putNumber(
                    f"{prefix}/Pose Y (m)", estimate.pose.y
                )
                wpilib.SmartDashboard.putNumber(
                    f"{prefix}/Pose Theta (deg)",
                    estimate.pose.rotation().degrees(),
                )
                wpilib.SmartDashboard.putNumber(
                    f"{prefix}/Latency (ms)",
                    estimate.latency_seconds * 1000.0,
                )
                wpilib.SmartDashboard.putNumber(
                    f"{prefix}/Avg Tag Distance (m)",
                    estimate.avg_tag_distance,
                )

        wpilib.SmartDashboard.putNumber(
            "Vision/Total Accepted Measurements",
            sum(self._vision_accepted_counts),
        )

    def getPose(self) -> Pose2d:
        """Return the current field-relative pose estimate."""
        return self._pose_estimator.getEstimatedPosition()

    def resetPose(self, pose: Pose2d) -> None:
        """Reset the pose estimator to a known pose."""
        self._pose_estimator.resetPosition(
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

        WPILib's SwerveDrive4PoseEstimator signatures require exactly four
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

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """Return the measured robot-relative chassis speeds.

        PathPlanner uses measured (not commanded) speeds for path-following
        feedback. The four module states are converted back through swerve
        kinematics to recover the chassis velocity.
        """
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            self.getModuleStates()
        )

    def setDriveVoltageForSysId(self, voltage: float) -> None:
        """Apply one open-loop drive voltage while holding wheels straight."""
        # Keep telemetry/physics from acting on a stale teleop command.
        self._desired_chassis_speeds = ChassisSpeeds()
        for module in self._modules:
            module.setSteeringAngle(Rotation2d())
            module.setDriveVoltage(voltage)

    def holdWheelsForSysId(self) -> None:
        """Keep the drive stopped while the steering loops align the wheels."""
        self.setDriveVoltageForSysId(0.0)

    def resetDriveControllers(self) -> None:
        """Clear drive PID state before starting a SysId test."""
        for module in self._modules:
            module.resetDriveController()

    def isDriveSysIdReady(self) -> bool:
        """Return True when all wheels are straight and nearly stopped."""
        return all(
            abs(module.getWheelAngle().radians())
            <= SysIdConstants.kSteeringToleranceRadians
            and abs(module.getDriveVelocity())
            <= SysIdConstants.kMaxStartSpeedMetersPerSecond
            for module in self._modules
        )

    def logDriveSysId(self, log: SysIdRoutineLog) -> None:
        """Log each drive module's voltage, distance, and speed for SysId."""
        for name, module in zip(
            self._sysid_module_names, self._modules
        ):
            log.motor(name).voltage(
                module.getDriveAppliedVoltage()
            ).position(
                module.getDrivePosition()
            ).velocity(
                module.getDriveVelocity()
            )

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
