# AGENTS.md

Agent guidance for the `try-robotpy` RobotPy project.

## Project overview

This is a small FRC RobotPy 2026 test project using the command-based framework.
It simulates an MK4i swerve drivetrain built from Spark MAX motor controllers,
NEO Vortex drive motors, and NEO 550 azimuth motors. The project targets the
2027 FRC season, with the roboRIO platform in mind.

- Season target: 2027 FRC
- Platform: roboRIO / RobotPy 2026.2.2
- Python: 3.14.0 locally; roboRIO 2026 requires Python 3.14
- Virtual environment: `.venv` in the project root

## Quick start

Activate the virtual environment before running any RobotPy command:

```powershell
.venv\Scripts\activate
```

Common commands:

| Action | Command |
|--------|---------|
| Sync dependencies to the roboRIO cache | `python -m robotpy sync` |
| GUI simulation | `python -m robotpy sim` |
| Headless simulation | `python -m robotpy sim --nogui` |
| Run unit tests | `python -m robotpy test` |

Important: use `python -m robotpy`, not the older `python robot.py sim` or
`wpilib.run(Robot)` entry points. Those are deprecated in RobotPy 2026.

## Project structure

```
try-robotpy/
├── .venv/                  # Python virtual environment
├── commands/               # Command-based commands
│   ├── __init__.py
│   └── swerve_drive.py     # Default swerve teleop command
├── constants.py            # Hardware constants for MK4i swerve
├── physics.py              # pyfrc physics engine (must be in project root)
├── pyproject.toml          # RobotPy dependencies and component configuration
├── robot.py                # Robot class (commands2.TimedCommandRobot)
├── subsystems/             # Subsystems
│   ├── __init__.py
│   ├── swerve_drive.py     # Swerve drivetrain (kinematics + odometry)
│   └── swerve_module.py    # Single MK4i swerve module
└── tests/
    ├── __init__.py
    └── test_basic.py       # Basic pyfrc tests
```

`physics.py` must stay in the project root. pyfrc looks for it there; placing
it under `sim/physics.py` causes a `Cannot enable physics support` error.

## Code style

- Declare types for function parameters and return values.
- Keep docstrings concise but include units and expected ranges where relevant.
- Write comments for obsure calculations or professional concept (e.g. PID, kinematics, odometry)
- Leave comments and references links on top of complex files.
- Prefer explicit configuration objects (for example `rev.SparkMaxConfig`) over
  multiple individual setter calls.
- Keep CAN IDs and other hardware constants in `constants.py` so they are easy to update when wiring changes.

## Testing

Run the test suite before committing:

```powershell
python -m robotpy test
```

For a quick behavioral smoke test without the GUI:

```powershell
python -m robotpy sim --nogui
```

Let it run for 10-20 seconds to confirm the robot initializes and the command
scheduler stays alive.

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
  (note the parentheses), not `Preset.REV_Vortex`.
- The idle-mode setter is `setIdleMode(...)`, not `idleMode(...)`.

### Physics / pyfrc
- `physics.py` must live in the project root, not `sim/physics.py`.
- Use units from `pyfrc.physics.units import units`; do not construct
  standalone `pint.Quantity` objects.

### Unit Tests
- NavX sim: getAngle() only reflects a written sim value after a sim tick; the test steps while disabled so physics.py (which runs only when enabled) doesn't overwrite it.
- robotpy test runs robot-fixture tests in isolated subprocesses and re-appends your pytest args. Passing a file path (e.g. -- tests/test_basic.py) makes each worker re-collect the whole file, creating two robots in one process → SparkMax instance already created. Use python -m robotpy test bare, or select with -k instead of file paths.