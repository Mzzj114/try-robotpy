# Offline recording workflow

Use this workflow when you have a saved `.ndjson` recording and the robot or sim is not running. It is ideal for whole-match forensics, transient issues you could not reproduce live, or post-event analysis.

Recordings are stored in the directory set by `NT_RECORDINGS_DIR` (default `./recordings`). They are plain NDJSON: each line is `{"time": float, "topic": str, "value": jsonable}`.

## 0. Produce a recording (if you do not have one yet)

Two ways, both writing the same NDJSON format — see `references/recording.md` for full details:

- `nt-recorder` CLI — standalone process, records until stopped or `--duration` elapses.
- `nt_subscribe(prefixes=[...], duration=N)` — the live MCP tool with its default `output="file"` captures a bounded window and returns a compact receipt instead of sample values:

  ```json
  {
    "connected": true,
    "output": "file",
    "recording_id": "nt-record-2026-09-15T143022Z.ndjson",
    "path": "recordings/nt-record-2026-09-15T143022Z.ndjson",
    "duration_seconds": 10.0,
    "rows": 847,
    "topic_count": 12,
    "topics": ["/SmartDashboard/Pose X", "..."],
    "topics_truncated": false,
    "truncated": false
  }
  ```

  The receipt contains no sample values — pass `recording_id` from it to the offline tools below to window into the capture.

## 1. Discover the telemetry

Search the codebase for `SmartDashboard`, `NetworkTables`, and `Field2d` to find which topics the robot publishes or subscribes to. For example:

- `subsystems/swerve_drive.py` publishing `/SmartDashboard/Swerve/Pose X`, `/SmartDashboard/Swerve/Pose Y`, `/SmartDashboard/Swerve/Pose Theta`, and a `Field2d` at `/SmartDashboard/Field`.
- Subsystems publishing encoder counts, voltages, setpoints, motor currents, etc.

## 2. Inspect the recording

1. `nt_list_recordings` — list files with size, `modified_iso`, and path. You usually want the most recent recording.
2. `nt_get_recording_info(recording_id=...)` — get duration, `total_samples`, and `topic_count` for a quick overview before opening it.
3. `nt_list_topics_offline(recording_id=...)` — list the topics present in the recording, optionally filtered by `prefix`, `regex`, or `wildcard`.

## 3. Replay samples

1. `nt_subscribe_offline(prefixes=[...], recording_id=...)` — replay samples from the recording. The response carries `recording_id`, `topics` (or `summary` with `format="summary"`), and always `rows`, `total_rows`, and `truncated`. Knobs:
   - `sample_interval` — decimate to one sample per N seconds.
   - `limit` — cap the entries returned **per topic** (default 1000).
   - `max_rows` — cap the total entries across topics (default 5000).
   - `format` — `"samples"` (default) or `"summary"` (min/max/mean/last).
   - `last_seconds` / `start` / `end` — window the replay.
   - A bare `"/"` prefix is refused — narrow it (e.g. `/SmartDashboard/`).
2. `nt_get_history(topic=..., recording_id=...)` — pull one topic's history. The response carries `topic`, `recording_id`, `rows`, `total_rows`, `truncated`, and `samples` (or `summary`). Windowing:
   - `last_seconds` — the most recent N seconds.
   - `start` / `end` — an explicit time window using NT4 timestamps.
   - `limit` — cap the returned entries (default 5000).

   Example: `nt_get_history(topic="/SmartDashboard/Swerve/Pose X", recording_id="nt-record-...ndjson", last_seconds=5)`.

If `truncated` is `true`, then `rows` < `total_rows`: you are looking at a subset, never treat it as complete. Narrow the prefix, shrink the time window, or raise the row budget. See the truncation section in `references/analysis-gotchas.md`.

## 4. Wrap up

1. Summarize the findings, referencing specific time windows or topic values when possible.
2. Ask the user to confirm or propose next steps (for example, a code fix or a follow-up live test).
