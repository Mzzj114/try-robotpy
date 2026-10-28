"""Drive SysId commands for the swerve drivetrain.

The drive motors are run open-loop with a common voltage while the steering
loops hold every wheel at 0 rad. The routine logs applied voltage, wheel
distance, and wheel speed for all four modules; analyze one representative
module (front-left is a good default) in SysId. ``robotInit`` must start
DataLogManager before these commands are used.

Run the tests from teleop with the driver controller (hold the button):

- A: quasistatic forward
- B: quasistatic reverse
- X: dynamic forward
- Y: dynamic reverse

Hold a quasistatic test for the full 10 s ramp. For a dynamic test, release
after roughly 2 s; holding it to the 10 s timeout will send the robot much
farther down the hallway.

References:
- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/creating-routine.html
- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/loading-data.html
"""

import commands2
from commands2.sysid import SysIdRoutine

from constants import SysIdConstants
from subsystems.swerve_drive import SwerveDrive


class SysIdCommands:
    """Builds the four drive characterization commands."""

    def __init__(self, swerve: SwerveDrive) -> None:
        """Construct the SysId routine for the swerve drive.

        :param swerve: The swerve drivetrain subsystem to characterize.
        """
        self._swerve = swerve
        self._routine = SysIdRoutine(
            SysIdRoutine.Config(
                rampRate=SysIdConstants.kRampRate,
                stepVoltage=SysIdConstants.kStepVoltage,
                timeout=SysIdConstants.kTimeout,
            ),
            SysIdRoutine.Mechanism(
                self._swerve.setDriveVoltageForSysId,
                self._swerve.logDriveSysId,
                self._swerve,
                SysIdConstants.kMechanismName,
            ),
        )

    def _prepare(self) -> commands2.Command:
        """Reset the drive loop and wait for straight, stationary wheels.

        There is deliberately no alignment timeout: if a steering module never
        reaches 0 rad, the test must not start with a turned wheel. Release the
        button to abort instead of collecting invalid data.
        """
        return commands2.InstantCommand(
            self._swerve.resetDriveControllers, self._swerve
        ).andThen(
            commands2.RunCommand(
                self._swerve.holdWheelsForSysId, self._swerve
            ).until(self._swerve.isDriveSysIdReady)
        )

    def quasistatic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        """Return a quasistatic forward or reverse test command."""
        return self._prepare().andThen(self._routine.quasistatic(direction))

    def dynamic(self, direction: SysIdRoutine.Direction) -> commands2.Command:
        """Return a dynamic forward or reverse test command."""
        return self._prepare().andThen(self._routine.dynamic(direction))
