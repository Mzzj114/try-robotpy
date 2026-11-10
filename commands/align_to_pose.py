

import math

import commands2
from wpilib import Timer
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.controller import PIDController

from constants import DriveConstants
from subsystems.swerve_drive import SwerveDrive

class AlignToPose(commands2.Command):
    def __init__(self, swerve: SwerveDrive, target: Pose2d):
        super().__init__()
        self.swerve = swerve
        self.target = target
        self.addRequirements(swerve)

        self.x = PIDController(3.0, 0.0, 0.0)
        self.y = PIDController(3.0, 0.0, 0.0)
        self.theta = PIDController(4.0, 0.0, 0.0)
        self.theta.enableContinuousInput(-math.pi, math.pi)

        self.timer = Timer()

    def initialize(self):
        self.x.reset()
        self.y.reset()
        self.theta.reset()
        self.timer.restart()

    def execute(self):
        pose = self.swerve.getPose()

        vx = self.x.calculate(pose.x, self.target.x)
        vy = self.y.calculate(pose.y, self.target.y)

        # Cap the vector, not each axis separately.
        speed = math.hypot(vx, vy)
        speed_cap = DriveConstants.kMaxSpeedMetersPerSecond / 7
        if speed > speed_cap:
            vx *= speed_cap / speed
            vy *= speed_cap / speed

        omega = self.theta.calculate(
            pose.rotation().radians(),
            self.target.rotation().radians(),
        )

        self.swerve.driveRobotRelative(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, omega, pose.rotation()
            )
        )

    def isFinished(self):
        pose = self.swerve.getPose()
        error = self.target.relativeTo(pose)
        heading_error = math.atan2(
            math.sin(error.rotation().radians()),
            math.cos(error.rotation().radians()),
        )
        return (
            abs(error.x) < 0.02
            and abs(error.y) < 0.02
            and abs(heading_error) < math.radians(1.5)
        ) or self.timer.hasElapsed(2.0)

    def end(self, interrupted):
        self.timer.stop()
        self.swerve.stop()