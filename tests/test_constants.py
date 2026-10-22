"""Sanity checks on the hardware constants in constants.py.

These guard the unit conversions and wiring-order assumptions the rest of the
drivetrain depends on. They are pure-Python and never touch the HAL.
"""

import math

import pytest

from constants import DriveConstants, ModuleConstants


def test_drive_encoder_factors_are_derived_from_wheel_and_gear_ratio() -> None:
    """Position factor converts motor rotations to meters, velocity to m/s."""
    expected_position = (
        DriveConstants.kWheelCircumferenceMeters / DriveConstants.kDriveGearRatio
    )
    assert DriveConstants.kDriveEncoderPositionFactor == pytest.approx(
        expected_position
    )
    assert DriveConstants.kDriveEncoderVelocityFactor == pytest.approx(
        expected_position / 60.0
    )


def test_wheel_circumference_matches_diameter() -> None:
    """Circumference is pi times the 4" wheel diameter."""
    assert DriveConstants.kWheelCircumferenceMeters == pytest.approx(
        DriveConstants.kWheelDiameterMeters * math.pi
    )


def test_module_locations_are_four_distinct_symmetric_corners() -> None:
    """The four module locations form a rectangle centered on the origin."""
    locations = DriveConstants.kModuleLocations
    assert len(locations) == 4

    half_wheel_base = DriveConstants.kWheelBase / 2.0
    half_track_width = DriveConstants.kTrackWidth / 2.0
    assert sorted(loc.x for loc in locations) == pytest.approx(
        [-half_wheel_base, -half_wheel_base, half_wheel_base, half_wheel_base]
    )
    assert sorted(loc.y for loc in locations) == pytest.approx(
        [-half_track_width, -half_track_width, half_track_width, half_track_width]
    )
    # No two modules share the same corner.
    assert len({(loc.x, loc.y) for loc in locations}) == 4


def test_can_id_tables_have_four_unique_ids() -> None:
    """Each module needs one drive, one turn, and one CANcoder ID."""
    id_tables = (
        ModuleConstants.kDriveMotorCanIds,
        ModuleConstants.kTurnMotorCanIds,
        ModuleConstants.kTurnCanCoderIds,
    )
    for ids in id_tables:
        assert len(ids) == 4
        assert len(set(ids)) == 4

    # A drive ID must never collide with a turn or CANcoder ID.
    all_ids = [can_id for ids in id_tables for can_id in ids]
    assert len(set(all_ids)) == 12


def test_angular_offsets_match_module_count() -> None:
    """There is exactly one calibration offset per module."""
    assert len(ModuleConstants.kAngularOffsets) == 4


def test_speed_limits_are_positive() -> None:
    """Kinematics and motion-profile limits must be usable by WPILib."""
    assert DriveConstants.kMaxSpeedMetersPerSecond > 0.0
    assert DriveConstants.kMaxAngularSpeed > 0.0
    assert DriveConstants.kModuleMaxAngularAcceleration > 0.0
