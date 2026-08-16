"""pyfrc physics engine for the differential drivetrain."""

import typing

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.drivetrains import FourMotorDrivetrain
from pyfrc.physics.units import units

if typing.TYPE_CHECKING:
    from robot import Robot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot") -> None:
        """Create the physics engine and keep a reference to the robot.

        Args:
            physics_controller: Interface used to update the simulation field.
            robot: The running robot instance, so we can read motor setpoints.
        """
        self.physics_controller = physics_controller
        self.robot = robot

        # Tune these to match your actual robot geometry and top speed.
        self.drivetrain = FourMotorDrivetrain(
            x_wheelbase=0.6 * units.meters,
            speed=3.0 * units.mps,
        )

    def update_sim(self, now: float, tm_diff: float) -> None:
        """Called every simulation loop to advance physics.

        Args:
            now: Current simulation time in seconds.
            tm_diff: Time elapsed since the last call in seconds.
        """
        drivetrain = self.robot.drivetrain

        # Read motor setpoints (-1.0 to 1.0) from the Spark MAX controllers.
        lf = drivetrain.left_leader.get()
        lr = drivetrain.left_follower.get()
        rf = drivetrain.right_leader.get()
        rr = drivetrain.right_follower.get()

        # Convert motor speeds to chassis speeds and update the field pose.
        speeds = self.drivetrain.calculate(lf, lr, rf, rr)
        self.physics_controller.drive(speeds, tm_diff)
