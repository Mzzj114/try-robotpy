# AGENTS.md

Agent guidance for the `try-robotpy` RobotPy project.

## Project overview

This is a small FRC RobotPy 2026 test project using the command-based framework.
It simulates and controls an MK4i swerve drivetrain built from Spark MAX motor
controllers, NEO Vortex drive motors, NEO 550 azimuth motors, and CTRE CANcoder
absolute encoders. The project targets the 2027 FRC season, with the roboRIO
platform in mind.

Current capabilities:

- Field-oriented and robot-relative swerve teleop.
- `SwerveDrive4PoseEstimator` odometry with three Limelight MegaTag2 cameras
  fused through a NavX gyro heading.
- PathPlanner autonomous routines and path following, configured through
  `deploy/pathplanner/`.
- Steering SysId characterization and live SmartDashboard PID/FF tuning.
- A NetworkTables debug console for on-robot Python evaluation (test only).

Key facts:

- Team number: 8326 (`.wpilib/wpilib_preferences.json`)
- Season target: 2027 FRC
- Platform: roboRIO / RobotPy 2026.2.2
- Python: 3.14.0 locally; roboRIO 2026 requires Python 3.14
- Virtual environment: `.venv` in the project root
- Vendor packages: `robotpy-rev==2026.0.4`, `phoenix6==26.3.0`,
  `robotpy-navx==2026.0.1.1`, `robotpy-pathplannerlib==2026.1.2`

## Quick start

Activate the virtual environment before running any RobotPy command:

```powershell
.venv\Scripts\activate
```

Common commands:

| Action | Command |
|--------|---------|
| Sync dependencies to the roboRIO cache | `python -m robotpy sync` |
| Deploy to the roboRIO | `python -m robotpy deploy` |
| GUI simulation | `python -m robotpy sim` |
| Headless simulation | `python -m robotpy sim --nogui` |
| Run unit tests | `python -m robotpy test` |
| Run selected tests | `python -m robotpy test -k <pattern>` |

Important: use `python -m robotpy`, not the older `python robot.py sim` or
`wpilib.run(Robot)` entry points. Those are deprecated in RobotPy 2026.

## Project structure

```
try-robotpy/
├── .agents/skills/frc-robotics-py/  # Project AI skill and FRC references
├── .venv/                  # Python virtual environment
├── commands/               # Command-based commands
│   ├── __init__.py
│   ├── module_test.py      # Manual single-module bring-up command
│   ├── swerve_drive.py     # Default swerve teleop command
│   ├── sysid.py            # Steering SysId commands
│   ├── tune_path_pid.py    # Live PathPlanner translation/rotation tuning
│   └── tune_pid.py         # Live drive/steering PID + steering FF tuning
├── constants.py            # Hardware, vision, SysId, auto, and OI constants
├── debug_console.py        # NetworkTables eval/exec console (test only)
├── deploy/pathplanner/     # PathPlanner settings, autos, paths, navgrid
├── limelight-maps/         # Custom .fmap files and coordinate notes
├── physics.py              # pyfrc physics engine (must be in project root)
├── pyproject.toml          # RobotPy dependencies and component configuration
├── requirements.txt        # pip freeze output (UTF-16 LE encoded)
├── robot.py                # Robot class (commands2.TimedCommandRobot)
├── subsystems/             # Subsystems
│   ├── __init__.py
│   ├── limelight.py        # NetworkTables MegaTag2 client
│   ├── swerve_drive.py     # Swerve drivetrain (kinematics + odometry + vision)
│   └── swerve_module.py    # Single MK4i swerve module
├── sysid-results.md        # Measured drive characterization results
└── tests/
    ├── __init__.py
    ├── test_basic.py       # Robot startup / mode smoke tests
    ├── test_constants.py   # Unit conversions and wiring-order checks
    ├── test_kinematics.py  # Swerve kinematics round-trip checks
    ├── test_swerve_sim.py  # Simulation-backed drivetrain and auto tests
    └── test_sysid.py       # Steering SysId command wiring smoke test
```

`physics.py` must stay in the project root. pyfrc looks for it there; placing
it under `sim/physics.py` causes a `Cannot enable physics support` error.

Generated or local-only paths are gitignored: `logs/`, `recordings/`,
`ctre_sim/`, `simgui*.json`, `.omo/`, and `opencode.jsonc`. Dashboard layouts
that are shared are tracked (`elastic-layout.json`, `networktables.json`).

## Architecture notes

- `robot.py` constructs `SwerveDrive`, sets the default `SwerveDriveCommand`
  (field-relative Xbox input), preloads `PathPlannerAuto` in `robotInit`, and
  registers the tuning commands under `Tune/` and `Test/` on SmartDashboard.
- `AutoBuilder.configure(...)` is called once in `SwerveDrive.__init__`, so the
  subsystem is the single owner of PathPlanner configuration.
- `SwerveDrive.periodic` refreshes each CANcoder once, updates the pose
  estimator, fuses vision, and updates the `Field2d` visualization every loop.
- Dashboard telemetry is deliberately slower than control:
  `publishTelemetry` runs at 10 Hz and `publishSlowTelemetry` at 1 Hz through
  `addPeriodic`. Keep new high-rate work out of these paths.
- Both swerve control loops run on the roboRIO and output voltage to the Spark
  MAX controllers. The Spark MAX onboard closed-loop controllers are not used.
- `physics.py` drives the simulated robot from the last commanded chassis
  speeds, then updates the simulated Spark MAX encoders, CANcoder positions, and
  NavX yaw so odometry tracks the physics pose.
- Drive motor characterization values live in `sysid-results.md` and are
  consolidated into `ModuleConstants` in `constants.py`.

## Code style

- Declare types for function parameters and return values.
- Keep docstrings concise but include units and expected ranges where relevant.
- Write comments for obscure calculations or professional concepts (e.g. PID,
  kinematics, odometry, vision fusion).
- Leave comments and reference links at the top of complex files.
- Prefer explicit configuration objects (for example `rev.SparkMaxConfig` and
  Phoenix 6 config classes) over multiple individual setter calls.
- Keep CAN IDs, Limelight names/IPs, and other hardware constants in
  `constants.py` so they are easy to update when wiring changes.
- Use SI units internally (meters, radians, seconds) and name constants with
  units, e.g. `kMaxSpeedMetersPerSecond`.
- Tuned gains belong back in `constants.py`; SmartDashboard edits only last for
  the current process.

## Testing

Run the test suite before committing:

```powershell
python -m robotpy test
```

The suite has three layers:

- Pure-Python checks (`test_constants.py`, `test_kinematics.py`) that never
  touch the HAL.
- pyfrc smoke tests (`test_basic.py`, `test_sysid.py`).
- Simulation-backed tests (`test_swerve_sim.py`) that run the real robot code
  under the simulated HAL and rely on `physics.py`.

For a quick behavioral smoke test without the GUI:

```powershell
python -m robotpy sim --nogui
```

Let it run for 10-20 seconds to confirm the robot initializes and the command
scheduler stays alive.

## Tuning and characterization

- `Tune/Tune Drive PID`, `Tune/Tune Turn PID`, and `Tune/Tune Turn FF` run a
  response on the front-left module and expose gains under `PID/Drive`,
  `PID/Turn`, and `PID/Turn FF`.
- `Tune/Tune Path Translation PID` and `Tune/Tune Path Rotation PID` follow
  `deploy/pathplanner/paths/Tune Translatoin PID.path` and
  `Tune Rotational PID.path`. The misspelling in `kTuneTranslationPathName` is
  the actual file name; update both together.
- Driver controller A/B also bind the two path-tuning commands.
- Steering SysId keybinds (A/B quasistatic, X/Y dynamic) and DataLogManager
  startup are currently commented out in `robot.py`. Re-enable
  `startDataloger()` and `configureSysidKeybinds()` before running SysId.
- The auto routine is `New Auto` from `deploy/pathplanner/autos`. PathPlanner
  robot geometry must stay in sync with `deploy/pathplanner/settings.json`
  because `RobotConfig.fromGUISettings()` reads that file.

## Known API gotchas

### Robot entry point

- Use `python -m robotpy sim`.
- GUI is the default; pass `--nogui` to disable it.
- Do **not** call `wpilib.run(Robot)` in `robot.py`.

### pyproject.toml

- `robotpy_extras` has been renamed to `components` under `[tool.robotpy]`.
- Vendor packages belong in `[tool.robotpy].requires` as a **list of strings**,
  not a `[tool.robotpy.requires]` key-value table.

### REV / Spark MAX

- `rev.ResetMode` and `rev.PersistMode` are top-level classes, not
  `rev.SparkBase.ResetMode`.
- `wpilib.MotorControllerGroup` is directly in `wpilib`, not
  `wpilib.motorcontroller`.
- `DriverStation.say()` does not exist; use `print()` for console output.
- Presets are callable methods: use `rev.SparkBaseConfig.Presets.REV_Vortex()`
  and `rev.SparkBaseConfig.Presets.REV_NEO_550()` (note the parentheses).
- The idle-mode setter is `setIdleMode(...)`, not `idleMode(...)`.
- For simulation, use `rev.SparkRelativeEncoderSim` for drive encoders.

### CTRE Phoenix 6 / CANcoder

- A CANcoder talks over CAN and cannot feed the Spark MAX data port, so the
  steering loop must live on the roboRIO.
- `get_absolute_position()` wraps each revolution; `get_position()` is
  continuous and is the signal to differentiate for SysId.
- Refresh multiple status signals with
  `BaseStatusSignal.refresh_all(...)` instead of calling blocking reads.
- Simulation writes use `cancoder.sim_state.set_raw_position(...)` and
  `set_velocity(...)`; Phoenix publishes the value after a sim tick.

### NavX

- `getAngle()` is clockwise-positive; `SwerveDrive.getRotation2d()` negates it
  to provide the CCW-positive heading WPILib expects.
- `zeroYaw()` preserves odometry translation.
- The NavX sim device is named `"navX-Sensor"` and is indexed by
  `getPort()` (`kMXP_SPI` is 0).

### Limelight / vision

- Read `botpose_orb` (the loaded `.fmap` frame). Do **not** use
  `botpose_orb_wpiblue` with a custom field map.
- MegaTag2 requires `robot_orientation_set` every frame. Yaw is CCW-positive,
  and 0 degrees must point along the loaded `.fmap` +X axis.
- Add each published frame to the pose estimator only once, keyed by
  `source_time_us`.
- MegaTag2 heading comes from the gyro, so `kVisionStdDevs` trusts translation
  but assigns a huge heading deviation.
- Frames are rejected while the robot spins faster than
  `kMaxAngularVelocityDps`.
- Limelight publishes `tv` as a double even though the docs say int.
- Camera NetworkTables names and IPs in `VisionConstants.kLimelights` must match
  the web UI, including each camera's robot-space pose (LL Forward/Right/Up/
  Roll/Pitch/Yaw).

### PathPlanner

- `PPHolonomicDriveController` has no public gain setters. Live tuning writes
  directly to its private `_xController`, `_yController`, and
  `_rotationController`; re-check this if the PathPlanner API changes.
- `RobotConfig.fromGUISettings()` reads `deploy/pathplanner/settings.json`, so
  the PathPlanner GUI remains the source of truth for robot geometry.
- Tuning commands disable path flipping so tuning always starts at the path's
  blue-origin pose.
- `PathPlannerAuto` is constructed in `robotInit`; `teleopInit` cancels it so a
  long auto cannot keep driving into teleop.

### Physics / pyfrc

- `physics.py` must live in the project root, not `sim/physics.py`.
- Use units from `pyfrc.physics.units import units`; do not construct
  standalone `pint.Quantity` objects.
- Physics only advances when the robot is enabled. Tests that write simulated
  sensor values directly must step while disabled so physics does not overwrite
  them.

### Unit tests

- NavX sim: `getAngle()` only reflects a written sim value after a sim tick;
  the test steps while disabled so `physics.py` (which runs only when enabled)
  does not overwrite it.
- robotpy test runs robot-fixture tests in isolated subprocesses and re-appends
  your pytest args. Passing a file path (e.g. `-- tests/test_basic.py`) makes
  each worker re-collect the whole file, creating two robots in one process
  and raising `SparkMax instance already created`. Use
  `python -m robotpy test` bare, or select with `-k` instead of file paths.
- CANcoder sim positions are in rotations and include the module angular
  offset. Step while disabled after writing them.

### Debug console

- `debug_console.py` exposes arbitrary remote code execution through
  `/Debug/Cmd` and `/Debug/Out` and is instantiated unconditionally in
  `robotInit`. It is test-only and must not run on a competition robot.
- NetworkTables suppresses a write whose value is unchanged, so re-send
  slightly different text to run the same command twice.
- Commands run on the main robot loop; blocking code stalls the robot.

## Current known limitations

- PathPlanner gains are initial guesses and still need real-robot tuning
  (`kTranslationP/D`, `kRotationP` in `AutoConstants`).
- `kMaxSpeedMetersPerSecond = 2.0` is a testing/feedforward value, not the
  drivetrain's true top speed.
- `requirements.txt` is generated by the RobotPy tooling and is UTF-16 LE
  encoded; prefer editing `pyproject.toml`.
