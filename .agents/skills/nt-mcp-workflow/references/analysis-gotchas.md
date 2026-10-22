# Analysis gotchas

Lessons learned the hard way during the team-8326 MK4i swerve steering investigation. These are observations from a real robot, not generic NetworkTables documentation. Your robot may behave differently.

## NDJSON recording shape

Every recording line is a three-column JSON object:

```json
{"time": 1.234, "topic": "/SmartDashboard/Pose X", "value": 1.5}
```

- `time`: float, NT4 timestamp in seconds.
- `topic`: string, the full topic path including any prefix.
- `value`: any JSON-serializable value (bool, int, float, string, list, dict).

There are no headers, no schema version field, and no metadata rows. The file is pure NDJSON.

## 1. Keys carry the `/SmartDashboard/` prefix

When values are published through `wpilib.SmartDashboard.putNumber(...)` (or `putString`, `putBoolean`, etc.), the topic name in NetworkTables includes the `/SmartDashboard/` prefix. A call like `SmartDashboard.putNumber("Pose X", 1.5)` creates the topic `/SmartDashboard/Pose X`, not `/Pose X`.

When filtering recordings or writing prefix lists, account for this. A prefix of `/SmartDashboard/` captures everything published through SmartDashboard. A prefix of `/` captures everything.

This caught us during the investigation: we initially searched for topic names without the prefix and found nothing.

## 2. Disabled windows are misleading

When a robot enters disabled mode, the default command does not run. The `stop()` method on subsystems zeroes motor outputs, and encoders freeze at their last position. The telemetry still streams: encoder values sit at a constant number, motor outputs sit at zero.

If you see a recording segment where encoder values are constant and motor outputs are zero, that is a disabled window, not a mechanical stall. Any conclusion like "the wheels settled at X degrees" must be checked against `Turn Output != 0`. If the output is zero, the motor was not trying to move. The encoder position at that point is the position where `stop()` was called, not where the wheel naturally settled.

During the MK4i investigation, we initially misread a disabled window as evidence that the steering was stuck at a fixed angle. The encoder was frozen, not stalled.

## 3. Slow sampling aliases the fast control loop

The robot's control loop runs at 50 Hz (20 ms period). The recording sample rate was 20 Hz (50 ms period). At this ratio, the recorder captures roughly one out of every 2.5 control cycles. Fast dynamics (oscillations, rapid corrections, sign changes in error) are aliased: they appear in the recording at a lower frequency and with distorted amplitudes.

Derivative-based tests are especially unreliable under aliasing. A test like `sign(error) == sign(deltaEncoder)` can fail spuriously because the recorder missed the control cycle where the sign flipped. Prefer magnitude evidence (does the encoder reach the target value at all?) and one-way-travel evidence (does the wheel sweep a full revolution, or does it bounce back?).

During the MK4i investigation, sign correlation tests gave contradictory results until we switched to checking encoder range and direction-of-travel instead.

## 4. Both NT3 and NT4 ports may be open

FRC robots often have both NT3 (port 1735) and NT4 (port 5810) servers running simultaneously. The `nt-mcp-server` and `nt-recorder` both default to port 5810 (NT4). If you are connecting to a real robot and getting no data, verify you are hitting port 5810, not 1735.

The NT4 protocol is the current standard and is what these tools speak. The NT3 port is there for legacy dashboard compatibility.

## 5. Never read a recording an active recorder is still writing

The offline reader functions (`get_history`, `subscribe_offline`, `list_recordings`, etc.) have no writer-detection. They open the file, read what is there, and return. If a `Recorder` process is still appending lines, the reader may see a partial file: some lines are fully written, others are mid-write, and the file size is still growing.

This can produce truncated results, JSON parse errors, or silently incomplete data. Always stop the recorder before reading its output file. If you are using the detached-on-Windows pattern, run the stop command first.

This is documented in the source at `nt_recorder.py` and was a real failure mode during the investigation: an agent tried to analyze a recording while the recorder was still running and got garbled results.

## 6. Truncation is explicit

When a tool returns a subset of the available data, it says so. The response includes:

- `truncated` (bool): true when more data exists than was returned.
- `rows` (int): number of rows actually returned.
- `total_rows` (int): total number of matching rows in the recording (only present when truncation metadata is enabled).

Never treat a clipped series as complete. If `truncated` is true, you are looking at a subset. Narrow your prefix, reduce the time window, or increase the row budget to see more.

During the investigation, we initially assumed a 500-row response was the full dataset. It was not. The `truncated` flag was there; we missed it.
