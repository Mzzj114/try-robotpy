"""Steering SysId commands for the swerve drivetrain.

The azimuth motors are run open-loop with a common voltage while the drive
motors are held at zero. The routine logs applied voltage, angle, and angular
speed for all four modules; analyze one representative module (front-left is
a good default) in SysId. ``robotInit`` must start DataLogManager before
these commands are used.

The drive motors were characterized separately; this file contains only the
steering routine so a steering log never contains drive data.

Run the tests from teleop with the driver controller (hold the button), with
the robot on blocks so the wheels can pivot freely:

- A: quasistatic forward
- B: quasistatic reverse
- X: dynamic forward
- Y: dynamic reverse

The steering tests are intentionally short because an azimuth module has no
hard stop. Running a forward/reverse pair roughly unwinds the module; if you
run only forward tests, unwind the modules by hand before the cables twist.

References:
- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/loading-data.html
- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/running-routine.html
"""

import commands2
from commands2.sysid import SysIdRoutine

from constants import SteeringSysIdConstants
from subsystems.swerve_drive import SwerveDrive


class SteeringSysIdCommands:
    """Builds the four steering (azimuth) characterization commands."""

    def __init__(self, swerve: SwerveDrive) -> None:
        """Construct the SysId routine for the steering motors.

        :param swerve: The swerve drivetrain subsystem to characterize.
        """
        self._swerve = swerve
        self._routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=SteeringSysIdConstants.kRampRate,
                stepVoltage=SteeringSysIdConstants.kStepVoltage,
                timeout=SteeringSysIdConstants.kTimeout,
            ),
            SysIdRoutine.Mechanism(
                self._swerve.setSteeringVoltageForSysId,
                self._swerve.logSteeringSysId,
                self._swerve,
                SteeringSysIdConstants.kMechanismName,
            ),
        )

    def _prepare(self) -> commands2.Command:
        """Stop the drive wheels and wait for the modules to settle.

        There is no timeout: if an encoder never reads near zero, release the
        button to abort rather than start a test from a moving module.
        """
        return commands2.RunCommand(
            self._swerve.stopForSteeringSysId, self._swerve
        ).until(self._swerve.isSteeringSysIdReady)

    def quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        """Return a quasistatic forward or reverse steering test command."""
        return self._prepare().andThen(
            self._routine.quasistatic(direction).withTimeout(
                SteeringSysIdConstants.kQuasistaticDuration
            )
        )

    def dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        """Return a dynamic forward or reverse steering test command."""
        return self._prepare().andThen(
            self._routine.dynamic(direction).withTimeout(
                SteeringSysIdConstants.kDynamicDuration
            )
        )
