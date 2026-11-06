"""Live PathPlanner path-following PID tuning commands.

Each command follows one path from ``deploy/pathplanner/paths`` and exposes
only the gains it is meant to tune on SmartDashboard. Edits are pushed to the
same ``PPHolonomicDriveController`` instance that AutoBuilder uses, so the
values carry over to autonomous routines for the rest of the session.

Workflow (one axis at a time):

1. Schedule the tuning command and watch the robot follow the path.
2. Drive the robot back to the path start, edit the gains on the dashboard,
   and schedule the command again.

The tuned values still need to be copied into ``constants.AutoConstants`` to
survive a robot restart.
"""

import math

import commands2
import wpilib
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.path import PathPlannerPath
from wpimath.controller import PIDController

from constants import AutoConstants
from subsystems.swerve_drive import SwerveDrive


class _TunePathPID(commands2.Command):
    """Follow a PathPlanner path while live-tuning one set of gains."""

    _PATH_NAME: str = ""
    _KEY_PREFIX: str = ""
    _GAIN_LABEL: str = ""

    def __init__(
        self,
        swerve: SwerveDrive,
        pids: tuple[PIDController, ...],
    ) -> None:
        """Construct the tuning command.

        :param swerve: The swerve drivetrain subsystem to tune.
        :param pids: The PathPlanner PID controllers that share these gains.
        """
        super().__init__()
        self._swerve = swerve
        self._pids = pids

        reference = pids[0]
        self._kP = reference.getP()
        self._kI = reference.getI()
        self._kD = reference.getD()

        self._key_p = f"{self._KEY_PREFIX}/P"
        self._key_i = f"{self._KEY_PREFIX}/I"
        self._key_d = f"{self._KEY_PREFIX}/D"
        wpilib.SmartDashboard.putNumber(self._key_p, self._kP)
        wpilib.SmartDashboard.putNumber(self._key_i, self._kI)
        wpilib.SmartDashboard.putNumber(self._key_d, self._kD)

        # Follow with the same controller AutoBuilder uses, but never flip the
        # path: tuning always starts from the path's blue-origin (0, 0).
        self._follow = FollowPathCommand(
            PathPlannerPath.fromPathFile(self._PATH_NAME),
            swerve.getPose,
            swerve.getRobotRelativeSpeeds,
            lambda speeds, _feedforwards: swerve.driveRobotRelative(speeds),
            swerve.getPathFollowingController(),
            swerve.getRobotConfig(),
            lambda: False,
            swerve,
        )

        self.addRequirements(swerve)
        print(
            f"[{type(self).__name__}] Initial {self._GAIN_LABEL} PID is "
            f"{self._kP} {self._kI} {self._kD}"
        )

    def initialize(self) -> None:
        """Start the underlying path-following command."""
        self._follow.initialize()

    def execute(self) -> None:
        """Apply edited gains, then run the path following control loop."""
        kP = wpilib.SmartDashboard.getNumber(self._key_p, self._kP)
        kI = wpilib.SmartDashboard.getNumber(self._key_i, self._kI)
        kD = wpilib.SmartDashboard.getNumber(self._key_d, self._kD)

        if (kP, kI, kD) != (self._kP, self._kI, self._kD):
            self._kP, self._kI, self._kD = kP, kI, kD
            for pid in self._pids:
                pid.setPID(kP, kI, kD)
            print(
                f"[{type(self).__name__}] Updated {self._GAIN_LABEL} PID to "
                f"{kP} {kI} {kD}"
            )

        self._follow.execute()
        self._publishMetrics()

    def _publishMetrics(self) -> None:
        """Publish per-axis tracking error; subclasses add the useful units."""

    def isFinished(self) -> bool:
        """Finish when the underlying path command finishes."""
        return self._follow.isFinished()

    def end(self, interrupted: bool) -> None:
        """Stop the chassis when tuning ends or is interrupted."""
        self._follow.end(interrupted)
        self._swerve.stop()


class TunePathTranslationPID(_TunePathPID):
    """Follow a path while tuning the PathPlanner translation (x/y) PID."""

    _PATH_NAME = AutoConstants.kTuneTranslationPathName
    _KEY_PREFIX = "PID/Path Translation"
    _GAIN_LABEL = "translation"

    def __init__(self, swerve: SwerveDrive) -> None:
        super().__init__(swerve, swerve.getPathTranslationPID())

    def _publishMetrics(self) -> None:
        x_pid, y_pid = self._pids
        pose = self._swerve.getPose()
        x_error = x_pid.getPositionError()
        y_error = y_pid.getPositionError()

        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Target X (m)", x_pid.getSetpoint()
        )
        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Target Y (m)", y_pid.getSetpoint()
        )
        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Pose X (m)", pose.x
        )
        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Pose Y (m)", pose.y
        )
        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Error X (m)", x_error
        )
        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Error Y (m)", y_error
        )
        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Error (m)", math.hypot(x_error, y_error)
        )


class TunePathRotationPID(_TunePathPID):
    """Follow a path while tuning the PathPlanner rotation PID."""

    _PATH_NAME = AutoConstants.kTuneRotationPathName
    _KEY_PREFIX = "PID/Path Rotation"
    _GAIN_LABEL = "rotation"

    def __init__(self, swerve: SwerveDrive) -> None:
        super().__init__(swerve, (swerve.getPathRotationPID(),))

    def _publishMetrics(self) -> None:
        rotation_pid = self._pids[0]
        pose = self._swerve.getPose()
        error_deg = math.degrees(rotation_pid.getPositionError())

        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Target (deg)",
            math.degrees(rotation_pid.getSetpoint()),
        )
        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Actual (deg)",
            pose.rotation().degrees(),
        )
        wpilib.SmartDashboard.putNumber(
            f"{self._KEY_PREFIX}/Error (deg)", error_deg
        )
