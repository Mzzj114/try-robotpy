# Offline recording workflow

Use this workflow when you have a saved `.ndjson` recording and the robot or sim is not running. It is ideal for whole-match forensics, transient issues you could not reproduce live, or post-event analysis.

Recordings are stored in the directory set by `NT_RECORDINGS_DIR` (default `./recordings`). The `nt-recorder` CLI writes them as plain NDJSON: each line is `{"time": float, "topic": str, "value": jsonable}`.

## 1. Discover the telemetry

Search the codebase for `SmartDashboard`, `NetworkTables`, and `Field2d` to find which topics the robot publishes or subscribes to. For example:

- `subsystems/swerve_drive.py` publishing `/SmartDashboard/Swerve/Pose X`, `/SmartDashboard/Swerve/Pose Y`, `/SmartDashboard/Swerve/Pose Theta`, and a `Field2d` at `/SmartDashboard/Field`.
- Subsystems publishing encoder counts, voltages, setpoints, motor currents, etc.

## 2. Inspect the recording

1. `nt_list_recordings` — list files with size, `modified_iso`, and path. You usually want the most recent recording.
2. `nt_get_recording_info(recording_id=...)` — get duration, `total_samples`, and `topic_count` for a quick overview before opening it.
3. `nt_list_topics_offline(recording_id=...)` — list the topics present in the recording, optionally filtered by `prefix`, `regex`, or `wildcard`.

## 3. Replay samples

1. `nt_subscribe_offline(prefixes=[...], recording_id=...)` — replay samples from the recording. Use the same token-saving knobs as live subscriptions:
   - `sample_interval` — decimate to one sample per N seconds.
   - `limit` — cap the number of returned samples.
   - `format` — `"samples"` (default) or `"summary"` (min/max/mean/last).
2. `nt_get_history(topic=..., recording_id=...)` — pull one topic's history with windowing:
   - `last_seconds` — the most recent N seconds.
   - `start` / `end` — an explicit time window using NT4 timestamps.

   Example: `nt_get_history(topic="/SmartDashboard/Swerve/Pose X", recording_id="nt-record-...ndjson", last_seconds=5)`.

## 4. Wrap up

1. Summarize the findings, referencing specific time windows or topic values when possible.
2. Ask the user to confirm or propose next steps (for example, a code fix or a follow-up live test).
