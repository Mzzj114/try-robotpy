"""Round-trip and scaling checks for the swerve kinematics.

Pure-Python: uses only the kinematics object built in constants.py.
"""

import pytest
from wpimath.kinematics import ChassisSpeeds

from constants import DriveConstants


def test_module_states_round_trip_back_to_chassis_speeds() -> None:
    """Inverse then forward kinematics returns the original velocity."""
    kinematics = DriveConstants.kDriveKinematics
    commanded = ChassisSpeeds(1.5, -0.75, 0.8)

    module_states = kinematics.toSwerveModuleStates(commanded)
    recovered = kinematics.toChassisSpeeds(module_states)

    assert recovered.vx == pytest.approx(commanded.vx)
    assert recovered.vy == pytest.approx(commanded.vy)
    assert recovered.omega == pytest.approx(commanded.omega)


def test_pure_forward_command_points_every_wheel_forward() -> None:
    """A +x chassis speed needs no steering and no per-wheel scaling."""
    module_states = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds(2.0, 0.0, 0.0)
    )
    for state in module_states:
        assert state.speed == pytest.approx(2.0)
        assert state.angle.radians() == pytest.approx(0.0)


def test_desaturation_caps_every_wheel_to_max_speed() -> None:
    """An over-speed command is scaled down without reversing direction."""
    kinematics = DriveConstants.kDriveKinematics
    too_fast = ChassisSpeeds(
        DriveConstants.kMaxSpeedMetersPerSecond * 3.0, 0.0, 0.0
    )

    module_states = kinematics.toSwerveModuleStates(too_fast)
    module_states = kinematics.desaturateWheelSpeeds(
        module_states, DriveConstants.kMaxSpeedMetersPerSecond
    )

    for state in module_states:
        assert state.speed <= DriveConstants.kMaxSpeedMetersPerSecond + 1e-9
        assert state.speed > 0.0
