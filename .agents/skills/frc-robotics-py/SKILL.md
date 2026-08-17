---
name: frc-robotics-py
description: Use when planning/developing/debugging robotpy related projects. Use when making a FRC robot in python.
license: MIT
metadata:
  author: Randy Z
  version: "1.0.0"
---

# Best Practice Developing a FRC Robot in Python

## 1. Where to Find Documentation

### Primary sources (use these)

| Sources | Access | Content | Reliability | Notes |
| :--- | :--- | :--- | :--- | :--- |
| wpilib | Context7 ID: `/websites/wpilib_en_stable` or URL: `https://docs.wpilib.org/en/stable/` | General knowledge and docs of everything, including conceptual knowledge, coding, and software usage. | High | This is a mixed guide for Java, C++, Python, and LabView. It doesn't have any vendor specific details nor API documentation. |
| robotpy docs | Use `webfetch` with `format=markdown` to fetch `https://robotpy.readthedocs.io/en/stable/index.html` and follow links in it for further discovery. | Specific and detailed docs for robotpy. It has links to vendors' python API too. | High | Context7 unavailable. |
| robotpy's REV lib API docs | Use `webfetch` with `format=markdown` to fetch `https://robotpy.readthedocs.io/projects/rev/en/stable/rev.html` and follow links in it for further discovery. | List of all classes and links to their doc page. | High | Context7 unavailable. |
| robotpy's Navx API docs | Use `webfetch` with `format=markdown` to fetch `https://robotpy.readthedocs.io/projects/navx/en/stable/api.html` and follow links in it for further discovery. | List of all classes and links to their doc page. | High | Context7 unavailable. |
| Phoenix 6 API docs | Use `webfetch` with `format=markdown` to fetch `https://api.ctr-electronics.com/phoenix6/stable/python/` and follow links in it for further discovery. | List of all classes and links to their doc page. | High | Context7 unavailable. |
| robotpy examples | This is a GitHub repo ([mostrobotpy](https://github.com/robotpy/mostrobotpy)) and examples are in `https://github.com/robotpy/mostrobotpy/tree/main/examples`. | Example codes for robotpy on various scenarios. | High | — |
| websearch | use your websearch tool to browse the internet | - | Low | Internet info may be outdated or not applicable, but this is a useful fallback for tricky issues. Check post date for search results. |


### Other Information Refernces
- [Quick intro to the competition](./references/first-robotics-competition.md)
- [How does code run on actual robot](./references/robot-architecture.md)
- [Coordinate System](./references/coordinate-conventions.md)
- RobotPy projects have less well-known structures and team specific conventions. These are usually written in `AGENTS.md`.


## 2. Trust Running Code Over Memory

RobotPy 2026 has several API changes from earlier versions and from Java WPILib. **Always verify by actually importing and running**:

In virtual environment,
```powershell
python -c "import rev; print([x for x in dir(rev) if 'Reset' in x])"
python -m robotpy test
python -m robotpy sim --nogui
```

If a class/constant feels off, inspect it directly:

```python
import wpilib
print([x for x in dir(wpilib) if 'MotorController' in x])
```

## 3. Make Clear Units
Keep Calculations in SI (meters, radians, seconds)
- All WPILib kinematics/odometry uses meters and radians
- Convert to SI once at the boundary (constants, sensor configs)
- Compute internally in SI, convert back only for logging/dashboard

Annotate Constants with Units in Names in the `k{Name}{Unit}` format (e.g., `kMaxSpeedMetersPerSecond`).

Use wpimath.units for Conversions
```python
from wpimath.units import inchesToMeters, feetToMeters, rotationsToRadians, rpmToRadiansPerSecond
# Convert between common FRC units
wheel_diameter = inchesToMeters(4.0)  # 4 inches → meters
max_speed = rotationsToRadians(1.0) / 60.0  # 1 RPM → rad/s
```

note that PID gains are unitless (but input/output must match units)

## 4. Suggested Workflow

1. Read the relevant file in this repo.
2. Read docs for conceptual info or API details if necessary
3. Make the smallest change that could work.
4. Run `python -m robotpy test` immediately.
5. If tests pass, run `python -m robotpy sim --nogui` for 10–20 seconds.