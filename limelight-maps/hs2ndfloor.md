# hs2ndfloor.fmap

Map for Limelight field-space localization testing on the high school 2nd floor. This is a custom/practice environment, not
an official FRC field, so the map origin is the measured reference point used on site.

This map and this document is confirmed by me that **it is consistent** on 
- coordinates
- real world
- limelight ui (botpose in field space)
During debug, the sensor inputs or code may indicate wrong magnitute or direction, so use this document as the base line.

## File format

- Top-level `type`: `frc`
- One entry per AprilTag in `fiducials`
- `family`: `apriltag3_36h11_classic`
- `size`: `165.1` mm
- `unique`: `1`
- `transform`: 4x4 row-major homogeneous matrix, SI units (meters). This is the
  tag pose in map space; the rotation columns are the tag axes expressed in map
  coordinates.

## Coordinate frame

Right-handed, standard FRC orientation:

- `+X` = direction you face when looking at the place that's suppose to be red alliance wall (in reality it's the 2nd floor hallway)
- `+Y` = your left (so the right-hand side wall is at negative `Y`)
- `+Z` = up
- Origin = the measured reference point on the 2nd floor (in reality a point we marked with tape)

All tags are vertical (mounted flat on walls), so each rotation is a pure yaw
about `Z`. The first rotation column is the tag's outward normal
`(cos yaw, sin yaw, 0)`.

## Measured tags

| ID | X (m)   | Y (m)    | Z (m)   | Yaw   | Normal | Wall              |
|----|---------|----------|---------|-------|--------|-------------------|
| 0  | 1.8669  | -1.30175 | 0.4445  | 90°   | +Y     | Right-hand wall   |
| 2  | 1.4605  | -1.30175 | 0.4445  | 90°   | +Y     | Right-hand wall   |
| 4  | -2.24155| -0.1143  | 0.71755 | 0°    | +X     | Back wall         |

Notes:

- Tags 0 and 2 share `y = -1.30175` and `z = 0.4445`, so they sit on the same
  wall running along `X` on the right-hand side, facing into the test area (`+Y`).
- Tag 4 sits on the back wall running along `Y` at `x = -2.24155`, facing into
  the test area (`+X`).
- `Z` is the measured tag center height above the floor.

## Transform construction

For a vertical tag at position `(x, y, z)` with yaw `yaw`:

```
[ cos(yaw)  -sin(yaw)  0  x ]
[ sin(yaw)   cos(yaw)  0  y ]
[    0          0      1  z ]
[    0          0      0  1 ]
```

Yaw is the direction the tag face points (its outward normal), measured from
`+X` counter-clockwise.