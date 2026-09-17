"""pyfrc physics engine for the MK4i swerve drivetrain.

The simulation uses the chassis speeds commanded by the SwerveDrive subsystem
(as computed from controller inputs or auto routines) to move the robot on the
simulated field. It also updates the simulated Spark MAX encoders and NavX gyro
so that the robot's odometry tracks the physics pose.
"""

import math
import typing

import rev
import wpilib
import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState

from constants import DriveConstants, ModuleConstants

if typing.TYPE_CHECKING:
    from robot import Robot


class _ModuleSim:
    """Helper that updates one module's simulated encoders."""

    def __init__(
        self,
        drive_motor: rev.SparkMax,
        turn_motor: rev.SparkMax,
        angular_offset: float,
    ) -> None:
        self.drive_encoder_sim = rev.SparkRelativeEncoderSim(drive_motor)
        self.turn_encoder_sim = rev.SparkAbsoluteEncoderSim(turn_motor)
        self.angular_offset = angular_offset
        self.last_turn_angle = 0.0
        self.drive_distance = 0.0

    def update(self, state: SwerveModuleState, dt: float) -> None:
        """Advance encoders to match the commanded module state.

        :param state: Desired wheel speed and angle from kinematics.
        :param dt: Simulation timestep in seconds.
        """
        # Apply the same optimization the real module uses so the simulated
        # wheel angle never jumps by pi.
        target = SwerveModuleState(state.speed, state.angle)
        target.optimize(Rotation2d(self.last_turn_angle))

        wheel_angle = target.angle.radians()

        # Estimate angular velocity from angle change before storing the new
        # angle so we compare against the previous timestep.
        if dt > 0.0:
            turn_velocity = _wrap_angle(wheel_angle - self.last_turn_angle) / dt
        else:
            turn_velocity = 0.0
        self.last_turn_angle = wheel_angle

        # The absolute encoder reading equals wheel angle plus the calibration
        # offset (same convention used in SwerveModule.getState).
        absolute_position = _wrap_angle(wheel_angle + self.angular_offset)
        self.turn_encoder_sim.setPosition(absolute_position)
        self.turn_encoder_sim.setVelocity(turn_velocity)

        # Integrate drive wheel distance.
        self.drive_distance += target.speed * dt
        self.drive_encoder_sim.setPosition(self.drive_distance)
        self.drive_encoder_sim.setVelocity(target.speed)


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "Robot") -> None:
        """Create the physics engine and keep a reference to the robot.

        Args:
            physics_controller: Interface used to update the simulation field.
            robot: The running robot instance.
        """
        self.physics_controller = physics_controller
        self.robot = robot

        modules = robot.swerve.getModules()
        self.module_sims = tuple(
            _ModuleSim(
                module.get_drive_motor(),
                module.get_turn_motor(),
                ModuleConstants.kAngularOffsets[i],
            )
            for i, module in enumerate(modules)
        )

        # NavX SimDevice created by navx.AHRS on kUSB1 is named navX-Sensor[2].
        self.gyro_sim = wpilib.simulation.SimDeviceSim("navX-Sensor[2]")
        self.gyro_yaw = self.gyro_sim.getDouble("Yaw")

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

        # Move the physics robot and get its new pose.
        pose = self.physics_controller.drive(chassis_speeds, tm_diff)

        # Update simulated encoders so odometry matches the physics pose.
        module_states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            chassis_speeds
        )
        for module_sim, state in zip(self.module_sims, module_states):
            module_sim.update(state, tm_diff)

        # Update simulated gyro heading. NavX getAngle() is clockwise-positive
        # degrees, while WPILib is counterclockwise-positive; the robot code
        # negates getAngle(), so we set Yaw to the negative of the CCW pose
        # rotation.
        self.gyro_yaw.set(-pose.rotation().degrees())
