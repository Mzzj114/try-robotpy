"""Live PID tuning commands for the swerve drivetrain.

Ported from last year's Java repo
(``2026-FRC-robot/src/main/java/frc/robot/commands/tunePID``).

Each command drives a step response on the front-left module and lets the
operator edit the P/I/D gains from SmartDashboard while it runs. Gains are
pushed to the roboRIO controllers live, so the step response can be watched
without redeploying.

Run them from SmartDashboard under ``Tune/``; they require the swerve
subsystem, so the default teleop command is interrupted while tuning.
"""

import commands2
import wpilib
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from subsystems.swerve_drive import SwerveDrive


class TuneDrivePID(commands2.Command):
    """Step the front-left wheel speed to tune the drive velocity PID."""

    _KEY_P = "PID/Drive/Tune Drive P"
    _KEY_I = "PID/Drive/Tune Drive I"
    _KEY_D = "PID/Drive/Tune Drive D"
    _KEY_TARGET = "PID/Drive/Target Speed"
    _KEY_ACTUAL = "PID/Drive/Actual Speed"
    _KEY_ERROR = "PID/Drive/Speed Error"

    _TARGET_SPEED = 2.0  # m/s step target
    _PERIOD = 4.0  # s; half the period at 0 m/s, half at the target

    def __init__(self, swerve: SwerveDrive) -> None:
        """Construct the drive tuning command.

        :param swerve: The swerve drivetrain subsystem to tune.
        """
        super().__init__()
        self._swerve = swerve

        # get the front right module
        self._module = swerve.getModules()[0]

        self._pid = self._module.get_drive_pid_controller()
        self._timer = wpilib.Timer()

        self._kP = self._pid.getP()
        self._kI = self._pid.getI()
        self._kD = self._pid.getD()

        wpilib.SmartDashboard.putNumber(self._KEY_P, self._kP)
        wpilib.SmartDashboard.putNumber(self._KEY_I, self._kI)
        wpilib.SmartDashboard.putNumber(self._KEY_D, self._kD)
        wpilib.SmartDashboard.putNumber(self._KEY_TARGET, 0.0)

        print(f"[TuneDrivePID] Initial PID is {self._kP} {self._kI} {self._kD}")

        self.addRequirements(swerve)

    def initialize(self) -> None:
        """Restart the step-response timer."""
        self._timer.restart()

    def execute(self) -> None:

        """Apply any edited gains, then command the next step target."""
        kP = wpilib.SmartDashboard.getNumber(self._KEY_P, self._kP)
        kI = wpilib.SmartDashboard.getNumber(self._KEY_I, self._kI)
        kD = wpilib.SmartDashboard.getNumber(self._KEY_D, self._kD)

        if (kP, kI, kD) != (self._kP, self._kI, self._kD):
            self._kP, self._kI, self._kD = kP, kI, kD
            self._pid.setPID(kP, kI, kD)
            print(f"[TuneDrivePID] Updated PID to: {kP} {kI} {kD}")

        # Step response: 0 m/s for the first half, target for the second half.
        if self._timer.get() % self._PERIOD < self._PERIOD / 2.0:
            target_speed = 0.0
        else:
            target_speed = self._TARGET_SPEED
        wpilib.SmartDashboard.putNumber(self._KEY_TARGET, target_speed)

        actual_speed = self._module.getState().speed
        wpilib.SmartDashboard.putNumber(self._KEY_ACTUAL, actual_speed)
        wpilib.SmartDashboard.putNumber(
            self._KEY_ERROR, target_speed - actual_speed
        )

        self._module.setDesiredState(
            SwerveModuleState(target_speed, Rotation2d())
        )

    def end(self, interrupted: bool) -> None:
        """Stop the chassis when tuning ends."""
        self._swerve.stop()


class TuneTurnPID(commands2.Command):
    """Step the front-left wheel angle to tune the steering PID."""

    _KEY_P = "PID/Turn/Tune P"
    _KEY_I = "PID/Turn/Tune I"
    _KEY_D = "PID/Turn/Tune D"
    _KEY_TARGET = "PID/Turn/Target Angle"
    _KEY_ACTUAL = "PID/Turn/Actual Angle"
    _KEY_ERROR = "PID/Turn/Angle Error"

    _TARGET_ANGLE = Rotation2d.fromDegrees(90.0)
    _PERIOD = 4.0  # s; half the period at 0 deg, half at the target

    def __init__(self, swerve: SwerveDrive) -> None:
        """Construct the steering tuning command.

        :param swerve: The swerve drivetrain subsystem to tune.
        """
        super().__init__()
        self._swerve = swerve
        self._module = swerve.getModules()[0]
        self._pid = self._module.get_turn_pid_controller()
        self._timer = wpilib.Timer()

        self._kP = self._pid.getP()
        self._kI = self._pid.getI()
        self._kD = self._pid.getD()

        wpilib.SmartDashboard.putNumber(self._KEY_P, self._kP)
        wpilib.SmartDashboard.putNumber(self._KEY_I, self._kI)
        wpilib.SmartDashboard.putNumber(self._KEY_D, self._kD)
        wpilib.SmartDashboard.putNumber(self._KEY_TARGET, 0.0)

        print(f"[TuneTurnPID] Initial PID is {self._kP} {self._kI} {self._kD}")

        self.addRequirements(swerve)

    def initialize(self) -> None:
        """Restart the step-response timer."""
        self._timer.restart()

    def execute(self) -> None:
        """Apply any edited gains, then command the next step target."""
        kP = wpilib.SmartDashboard.getNumber(self._KEY_P, self._kP)
        kI = wpilib.SmartDashboard.getNumber(self._KEY_I, self._kI)
        kD = wpilib.SmartDashboard.getNumber(self._KEY_D, self._kD)

        if (kP, kI, kD) != (self._kP, self._kI, self._kD):
            self._kP, self._kI, self._kD = kP, kI, kD
            self._pid.setPID(kP, kI, kD)
            print(f"[TuneTurnPID] Updated PID to: {kP} {kI} {kD}")

        # Step response: 0 deg for the first half, target for the second half.
        if self._timer.get() % self._PERIOD < self._PERIOD / 2.0:
            target_angle = Rotation2d()
        else:
            target_angle = self._TARGET_ANGLE
        wpilib.SmartDashboard.putNumber(
            self._KEY_TARGET, target_angle.degrees()
        )

        # getState().angle is the wheel angle (offset already removed), the
        # same frame the step target is expressed in.
        actual_angle = self._module.getState().angle
        error = target_angle - actual_angle
        wpilib.SmartDashboard.putNumber(
            self._KEY_ACTUAL, actual_angle.degrees()
        )
        wpilib.SmartDashboard.putNumber(self._KEY_ERROR, error.degrees())

        self._module.setDesiredState(SwerveModuleState(0.1, target_angle))

    def end(self, interrupted: bool) -> None:
        """Stop the chassis when tuning ends."""
        self._swerve.stop()
