"""Simple four-wheel differential drivetrain using Spark MAX + NEO Vortex."""

import commands2
import rev
import wpilib
import wpilib.drive


class Drivetrain(commands2.Subsystem):
    """Differential drivetrain with two Spark MAX leaders and two followers."""

    # CAN IDs - adjust these to match your robot's wiring
    LEFT_LEADER_ID = 1
    LEFT_FOLLOWER_ID = 2
    RIGHT_LEADER_ID = 3
    RIGHT_FOLLOWER_ID = 4

    def __init__(self) -> None:
        super().__init__()

        # Create motor controllers. NEO Vortex is brushless.
        self.left_leader = rev.SparkMax(
            self.LEFT_LEADER_ID, rev.SparkLowLevel.MotorType.kBrushless
        )
        self.left_follower = rev.SparkMax(
            self.LEFT_FOLLOWER_ID, rev.SparkLowLevel.MotorType.kBrushless
        )
        self.right_leader = rev.SparkMax(
            self.RIGHT_LEADER_ID, rev.SparkLowLevel.MotorType.kBrushless
        )
        self.right_follower = rev.SparkMax(
            self.RIGHT_FOLLOWER_ID, rev.SparkLowLevel.MotorType.kBrushless
        )

        # Build a configuration tuned for NEO Vortex.
        config = (
            rev.SparkMaxConfig()
            .apply(rev.SparkBaseConfig.Presets.REV_Vortex())
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(60)
            .openLoopRampRate(0.1)
            .voltageCompensation(12.0)
        )

        # Apply config to all controllers. Reset safe parameters so we start
        # from a known state, but do not persist to flash during development.
        for spark in (
            self.left_leader,
            self.left_follower,
            self.right_leader,
            self.right_follower,
        ):
            spark.configure(
                config,
                rev.ResetMode.kResetSafeParameters,
                rev.PersistMode.kNoPersistParameters,
            )

        # Followers mirror their side leader.
        self.left_follower.configure(
            rev.SparkMaxConfig().follow(self.left_leader),
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )
        self.right_follower.configure(
            rev.SparkMaxConfig().follow(self.right_leader),
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

        # One side is typically inverted so forward is forward for the robot.
        self.right_leader.configure(
            rev.SparkMaxConfig().inverted(True),
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

        # Combine each side into a motor controller group for WPILib.
        self.left_motors = wpilib.MotorControllerGroup(
            self.left_leader, self.left_follower
        )
        self.right_motors = wpilib.MotorControllerGroup(
            self.right_leader, self.right_follower
        )

        # DifferentialDrive handles arcade/tank curvature math.
        self.drive = wpilib.drive.DifferentialDrive(
            self.left_motors, self.right_motors
        )

        # Safer teleop feel: square inputs to give finer low-speed control.
        self.drive.setDeadband(0.05)

    def arcadeDrive(self, forward: float, rotation: float) -> None:
        """Drive with arcade controls.

        Args:
            forward: Throttle input, -1 (full reverse) to 1 (full forward).
            rotation: Rotation input, -1 (full left) to 1 (full right).
        """
        self.drive.arcadeDrive(forward, rotation, squareInputs=True)

    def tankDrive(self, left_speed: float, right_speed: float) -> None:
        """Drive with tank controls.

        Args:
            left_speed: Left side throttle, -1 to 1.
            right_speed: Right side throttle, -1 to 1.
        """
        self.drive.tankDrive(left_speed, right_speed, squareInputs=True)

    def stop(self) -> None:
        """Stop the drivetrain."""
        self.drive.stopMotor()

    def periodic(self) -> None:
        """Send telemetry to SmartDashboard."""
        wpilib.SmartDashboard.putNumber("Drivetrain/LeftSpeed", self.left_leader.get())
        wpilib.SmartDashboard.putNumber(
            "Drivetrain/RightSpeed", self.right_leader.get()
        )
