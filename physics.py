"""pyfrc physics engine for the MK4i swerve drivetrain.

The simulation uses the chassis speeds commanded by the SwerveDrive subsystem
(as computed from joystick inputs or auto routines) to move the robot on the
simulated field. This avoids needing full Spark MAX SimState models for a demo.
"""

import typing

import wpilib
from pyfrc.physics.core import PhysicsInterface

if typing.TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot") -> None:
        """Create the physics engine and keep a reference to the robot.

        Args:
            physics_controller: Interface used to update the simulation field.
            robot: The running robot instance.
        """
        self.physics_controller = physics_controller
        self.robot = robot

    def update_sim(self, now: float, tm_diff: float) -> None:
        """Called every simulation loop to advance physics.

        Args:
            now: Current simulation time in seconds.
            tm_diff: Time elapsed since the last call in seconds.
        """
        if not wpilib.DriverStation.isEnabled():
            return

        swerve = self.robot.swerve
        chassis_speeds = swerve.getDesiredChassisSpeeds()

        self.physics_controller.drive(chassis_speeds, tm_diff)
