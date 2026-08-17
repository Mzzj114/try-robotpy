# Coordinate Conventions for FRC RobotPy

Reference: [WPILib Coordinate System](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system) | [Limelight 3D Coordinate Systems](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems)

---

## Origin Points by Frame

| Frame | Origin Location | Notes |
|-------|----------------|-------|
| **WPILib Robot** | Center of robot projected to floor | All kinematics use this |
| **WPILib Field (Always Blue)** | Blue alliance origin | X+ points away from blue wall |
| **Limelight Camera** | Camera lens | Physical sensor center |
| **Limelight Robot** | Center of robot projected to floor | Same as WPILib robot origin |
| **Limelight Target** | Center of AprilTag | Z+ points out from tag plane |
| **Limelight Field (WPIBlue)** | Blue alliance origin | Identical to WPILib field |

**Key rule**: WPILib Field and Limelight WPIBlue share the **same origin** (blue alliance origin). When using AprilTags, always use the **Always Blue** origin approach — no alliance-dependent origin switching required.

---

## WPILib NWU Convention (Standard)

| Axis | Robot Frame | Field Frame |
|------|------------|-------------|
| **+X** | Forward | Toward opponent alliance station |
| **+Y** | Left | Toward left field boundary |
| **+Z** | Up | Up |

**Rotation**: CCW positive when viewed from above. 0° = +X axis.
**Range**: (-180°, 180°] or (-π, π].

```python
from wpimath.geometry import Rotation2d
forward = Rotation2d(0)           # 0°
left    = Rotation2d(math.pi/2)   # 90° CCW
right   = Rotation2d(-math.pi/2)  # -90°
back    = Rotation2d(math.pi)     # 180°
```

---

## Limelight Coordinate Systems

### Limelight Camera Space
| Axis | Direction |
|------|-----------|
| X+ | Right (from camera view) |
| Y+ | Down |
| Z+ | Out of camera lens |

### Limelight Robot Space
| Axis | Direction | WPILib Match |
|------|-----------|-------------|
| X+ | Forward | ✅ Same |
| Y+ | **Right** | ⚠️ **Opposite** (WPILib is Left) |
| Z+ | Up | ✅ Same |

### Limelight Field Space — **Use FRC WPIBlue**
| Axis | Direction | WPILib Match |
|------|-----------|-------------|
| X+ | Toward opponent | ✅ Same |
| Y+ | Toward left | ✅ Same |
| Z+ | Up | ✅ Same |
| Rotation | CCW positive | ✅ Same |

**WPIBlue is identical to WPILib's field frame. No conversion needed.**

---

## Practical Conversions

### Robot Space (Limelight) → WPILib Robot Frame
```python
# If you receive robot-relative pose from Limelight in its Robot Space:
wpilib_x = ll_tx        # Forward matches
wpilib_y = -ll_ty       # Right → Left (NEGATE)
wpilib_z = ll_tz        # Up matches
```

### MegaTag2 Pose Integration
```python
from wpimath.geometry import Pose3d, Rotation3d, Translation3d

# Read from NetworkTables — ALREADY in WPIBlue field frame
botpose = limelight_table.getNumberArray("botpose_wpiblue", [])
if len(botpose) >= 6:
    x, y, z, roll, pitch, yaw = botpose[:6]
    camera_pose = Pose3d(
        Translation3d(x, y, z),
        Rotation3d(roll, pitch, yaw)
    )
    # Transform to robot center
    robot_pose = camera_pose.transformBy(robot_to_camera.inverse())
    pose_estimator.addVisionMeasurement(robot_pose.toPose2d(), timestamp)
```

### Camera Mount Calibration (in WPILib robot frame)
```python
# +X forward, +Y left, +Z up
robot_to_camera = Pose3d(
    Translation3d(0.20, 0.05, 0.40),      # x, y, z
    Rotation3d(0, math.radians(-15), 0)   # roll, pitch, yaw
)
```

---

## Swerve Module Angle

| Value | Direction |
|-------|-----------|
| 0 rad | Module facing forward (robot +X) |
| +π/2 | Module facing left (robot +Y) |
| -π/2 | Module facing right (robot -Y) |

```python
from wpimath.kinematics import SwerveModuleState
state = SwerveModuleState(speed=2.0, angle=Rotation2d(0))  # Forward
```

---

## Quick Reference: Frame Alignment

| System | X+ | Y+ | Z+ | Rotation |
|--------|-----|-----|-----|----------|
| WPILib Robot | Fwd | Left | Up | CCW+ |
| WPILib Field | Fwd | Left | Up | CCW+ |
| LL Camera | Right | Down | Out | — |
| LL Robot | Fwd | **Right** | Up | — |
| LL Field (WPIBlue) | Fwd | Left | Up | CCW+ ✅ |

---

## Common Gotchas

- **NavX**: Reports CW positive. Use `navx.getRotation2d()` (handles negation) or negate `getAngle()` manually.
- **Absolute Encoders**: Often 0→1 rotations CW. Convert: `angle_rad = -raw * 2π + offset`.
- **Swerve Kinematics**: Module locations use robot center origin, +X forward, +Y left.
- **Limelight `botpose_wpiblue`**: Ready for WPILib — just transform by camera offset.
- **Limelight `botpose` (no suffix)**: Legacy, avoid — frame may differ.
