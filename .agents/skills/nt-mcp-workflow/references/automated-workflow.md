# Automated live workflow

Use this workflow for focused, interactive investigations where you can control the robot or sim through NetworkTables in real time. Examples: tuning a PID, verifying a sensor value, or driving a single subsystem through a test routine.

## 1. Discover the telemetry

Search the codebase for `SmartDashboard`, `NetworkTables`, and `Field2d` to find which topics the robot publishes or subscribes to. Common targets:

- `subsystems/swerve_drive.py` publishing `/SmartDashboard/Swerve/Pose X`, `/SmartDashboard/Swerve/Pose Y`, `/SmartDashboard/Swerve/Pose Theta`, and a `Field2d` at `/SmartDashboard/Field`.
- Subsystems publishing encoder counts, voltages, setpoints, motor currents, etc.

If the existing telemetry is not enough to observe or control the behavior you are debugging, create a dedicated testing branch and add fields:

- **Read-only diagnostics**: extra `putNumber` / `putBoolean` / `putStringArray` calls so the behavior is visible.
- **Writable controls**: fields the MCP can set to operate the robot. Place these under a `/MCP` prefix, for example `/SmartDashboard/MCP/testArm/targetAngle`, `/SmartDashboard/MCP/testMotor/run`, so they are easy to find and clearly separated from production telemetry.

## 2. Bring up the NT source

- **RobotPy sim**: Start it detached so it does not block your tool calls. In PowerShell, use `Start-Process` instead of a foreground launch:

  ```powershell
  .venv\Scripts\activate
  Start-Process -FilePath "python" -ArgumentList "-m", "robotpy", "sim" `
      -WorkingDirectory "D:\Program\Hardware\try-robotpy"
  ```

- **Real robot**: The user must connect the cable or Wi-Fi so you can reach the roboRIO, while a second adapter keeps the internet connection — see `references/network-setup.md` for the two-adapter setup and static routing. Confirm the team number or IP before connecting.

Wait a few seconds for the robot/sim to boot, then proceed with the chosen workflow reference.


## 3. Connect and inspect

1. `nt_connect` — start the NT4 client. Defaults to `127.0.0.1:5810` for the sim.
2. `nt_connection_info` — confirm the remote IP and that the link is up.
3. `nt_list_topics` — list available topics, optionally filtered by `prefix`.
4. `nt_get_info` — inspect a topic's type and properties before writing, so you know whether it is a `double`, `string`, `double[]`, etc.
5. `nt_get` / `nt_get_multiple` — read current values.

## 4. Drive and observe

1. `nt_set` / `nt_set_multiple` — publish inputs such as a `/SmartDashboard/MCP/*` setpoint the robot reads.
2. `nt_subscribe` — watch telemetry for a bounded duration. To keep responses small and avoid wasting context tokens:
   - `sample_interval` — decimate to one sample per N seconds.
   - `change_only` — skip samples that barely changed (great for mostly-static values; a spinning pose still streams).
   - `limit` — cap the number of returned samples.
   - `duration` — how long to collect.

   Example: `nt_subscribe(prefixes=["/SmartDashboard/Swerve"], duration=3, sample_interval=0.5, change_only=true)`.

3. `nt_disconnect` — tear down the connection when finished.

## 5. Wrap up

1. Summarize the results briefly.
2. Ask the user to confirm.
3. If the user approves, you may leave extra files you created (for example, `commands/tune_shooter_pid.py`) for future reference, but restore core files such as `robot.py` to their original state.
4. Commit and merge the test branch if necessary.

## Example: tuning a single-motor shooter PID

### Robot code changes

1. Create a `tune_shooter_pid` command.
2. In robot periodic, add code to run or stop the command based on `/SmartDashboard/MCP/tuneShooterPID/run` (boolean).
3. In the command, add code to spin the shooter motor using:
   - `/SmartDashboard/MCP/tuneShooterPID/target` for the target velocity.
   - `/SmartDashboard/MCP/tuneShooterPID/P`, `/I`, `/D` for the PID gains.
4. Add the actual velocity measurement to telemetry, and optionally the error (`target - actual`).

### MCP tuning loop

1. Connect to the robot.
2. Set new P, I, D values and a target velocity.
3. Start subscribing to the actual velocity topic.
4. Start the command by setting `/SmartDashboard/MCP/tuneShooterPID/run` to `true`.
5. Let it run for a few seconds, then set `run` to `false`.
6. Poll the subscription and review the response.
7. If more tuning is needed, go back to step 2; otherwise stop and clean up.
