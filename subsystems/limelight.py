"""NetworkTables interface to a Limelight running MegaTag2 localization.

Limelight is a NetworkTables 4 client, so its data is published under a table
named after the camera (the "NetworkTables Name" in the web UI). No vendor
library is required; this module reads the documented NT keys directly.

MegaTag2 needs the robot's field-relative yaw every frame. Publish it with
``set_robot_orientation`` before reading ``botpose_orb_wpiblue``.

References:
- MegaTag2: https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization-megatag2
- NetworkTables API: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
"""

import time
from dataclasses import dataclass
from typing import Optional

import ntcore
from wpimath.geometry import Pose2d, Rotation2d


@dataclass(frozen=True)
class MegaTag2Estimate:
    """One MegaTag2 pose measurement in WPILib blue-origin field space."""

    pose: Pose2d
    timestamp_seconds: float
    source_time_us: int
    latency_seconds: float
    tag_count: int
    tag_span: float
    avg_tag_distance: float
    avg_tag_area: float


class Limelight:
    """Reads MegaTag2 localization data from one Limelight over NetworkTables."""

    # Index layout of the Limelight botpose arrays.
    _X = 0
    _Y = 1
    _Z = 2
    _ROLL = 3
    _PITCH = 4
    _YAW = 5
    _LATENCY_MS = 6
    _TAG_COUNT = 7
    _TAG_SPAN = 8
    _AVG_TAG_DISTANCE = 9
    _AVG_TAG_AREA = 10
    _MIN_LENGTH = 11

    def __init__(self, table_name: str, ip: str = "") -> None:
        """Connect to one camera's NetworkTables table.

        :param table_name: Camera name configured in the Limelight web UI.
        :param ip: Camera IP, used for diagnostics only.
        """
        self.table_name = table_name
        self.ip = ip

        table = ntcore.NetworkTableInstance.getDefault().getTable(table_name)
        self._pose_entry = table.getEntry("botpose_orb_wpiblue")
        self._orientation_entry = table.getEntry("robot_orientation_set")
        self._heartbeat_entry = table.getEntry("hb")
        self._target_valid_entry = table.getEntry("tv")

        self._heartbeat = 0.0
        self._heartbeat_changed_at = -1.0

    def set_robot_orientation(
        self,
        yaw_degrees: float,
        yaw_rate_dps: float = 0.0,
        pitch_degrees: float = 0.0,
        pitch_rate_dps: float = 0.0,
        roll_degrees: float = 0.0,
        roll_rate_dps: float = 0.0,
    ) -> None:
        """Publish the robot orientation MegaTag2 uses for this frame.

        The array is ``[yaw, yawRate, pitch, pitchRate, roll, rollRate]`` in
        degrees and degrees per second. MegaTag2 is CCW-positive, with 0
        degrees facing the red alliance wall.
        """
        self._orientation_entry.setDoubleArray(
            [
                yaw_degrees,
                yaw_rate_dps,
                pitch_degrees,
                pitch_rate_dps,
                roll_degrees,
                roll_rate_dps,
            ]
        )

    def get_megatag2_estimate(self) -> Optional[MegaTag2Estimate]:
        """Return the latest MegaTag2 measurement, or None if unavailable.

        ``source_time_us`` identifies the frame. Callers should use it to add
        each published frame to the pose estimator only once.
        """
        # Read the value and its timestamp atomically so the pose cannot be
        # paired with a newer frame's timestamp.
        value = self._pose_entry.getValue()
        if not value.isValid() or not value.isDoubleArray():
            return None
        raw = value.getDoubleArray()
        if len(raw) < self._MIN_LENGTH:
            return None

        tag_count = int(raw[self._TAG_COUNT])
        if tag_count <= 0:
            return None

        source_time_us = value.time()
        if source_time_us <= 0:
            return None

        latency_seconds = raw[self._LATENCY_MS] / 1000.0

        # The NT server timestamp is in microseconds on the roboRIO clock.
        # Subtract the camera latency to approximate the frame capture time.
        timestamp_seconds = source_time_us / 1e6 - latency_seconds

        return MegaTag2Estimate(
            pose=Pose2d(
                raw[self._X],
                raw[self._Y],
                Rotation2d.fromDegrees(raw[self._YAW]),
            ),
            timestamp_seconds=timestamp_seconds,
            source_time_us=source_time_us,
            latency_seconds=latency_seconds,
            tag_count=tag_count,
            tag_span=raw[self._TAG_SPAN],
            avg_tag_distance=raw[self._AVG_TAG_DISTANCE],
            avg_tag_area=raw[self._AVG_TAG_AREA],
        )

    def update_status(self) -> tuple[float, bool]:
        """Refresh the heartbeat and report whether the camera is alive.

        The heartbeat increments once per processed frame, so a changing value
        proves the camera is alive. Returns ``(heartbeat, connected)``.
        """
        heartbeat = self._heartbeat_entry.getDouble(0.0)
        if heartbeat != self._heartbeat:
            self._heartbeat = heartbeat
            self._heartbeat_changed_at = time.monotonic()

        connected = (
            self._heartbeat_changed_at >= 0.0
            and time.monotonic() - self._heartbeat_changed_at < 2.0
        )
        return heartbeat, connected

    def has_target(self) -> bool:
        """Return True when the camera reports a valid target."""
        # Limelight publishes tv as a double even though the docs say int.
        return self._target_valid_entry.getDouble(0.0) > 0.0
