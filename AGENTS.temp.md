# RobotPy 测试环境搭建记录

---

## 1. 环境信息

| 项目 | 值 |
|------|-----|
| 操作系统 | Windows (win32) |
| Python 版本 | 3.14.0 |
| 虚拟环境 | `.venv`（位于项目根目录） |
| RobotPy 版本 | 2026.2.2 |
| 目标赛季 | 2027 FRC |

> 说明：原计划推荐 Python 3.12，但本机已安装 3.14.0。根据 report.md，roboRIO 2026 仅支持 Python 3.14，因此 3.14 反而与目标平台一致，无需降级。

---

## 2. 快速开始

### 2.1 激活虚拟环境

```powershell
.venv\Scripts\activate
```

### 2.2 常用命令

| 操作 | 命令 |
|------|------|
| 同步依赖到 roboRIO 缓存 | `python -m robotpy sync` |
| GUI 仿真（默认） | `python -m robotpy sim` |
| 无 GUI 仿真 | `python -m robotpy sim --nogui` |
| 运行单元测试 | `python -m robotpy test` |

> 注意：旧教程中常见的 `python robot.py sim` 和 `wpilib.run(Robot)` 在 2026 版本中已弃用，应统一使用 `python -m robotpy`。

---

## 3. 项目结构

```
try-robotpy/
├── .venv/                  # Python 虚拟环境
├── commands/               # 命令目录
│   ├── __init__.py
│   └── arcade_drive.py     # Arcade 驱动命令
├── physics.py              # pyfrc 物理引擎（位于根目录，pyfrc 默认查找位置）
├── pyproject.toml          # RobotPy 依赖与组件配置
├── robot.py                # 机器人主类
├── setup.md                # 本文件
├── subsystems/             # 子系统目录
│   ├── __init__.py
│   └── drivetrain.py       # 四轮差速底盘（Spark MAX + NEO Vortex）
└── tests/
    ├── __init__.py
    └── test_basic.py       # 基础 pyfrc 测试
```

> 注意：`physics.py` 必须放在项目根目录。如果放在 `sim/physics.py`，pyfrc 会报 `Cannot enable physics support, ...\physics.py not found`。


## 4. Known API Gotchas

### Starting the robot
- Use `python -m robotpy sim`, **not** `python robot.py sim`.
- GUI is the default; use `--nogui` to disable it.
- Do **not** put `wpilib.run(Robot)` in `robot.py`.

### `pyproject.toml`
- `robotpy_extras` is now `components`.
- Vendor packages go in `[tool.robotpy].requires` as a **list of strings**, not a `[tool.robotpy.requires]` key-value table.

### REV / Spark MAX
- `rev.ResetMode` and `rev.PersistMode` are top-level classes, **not** `rev.SparkBase.ResetMode`.
- `wpilib.MotorControllerGroup` is directly in `wpilib`, not `wpilib.motorcontroller`.
- `DriverStation.say()` does not exist; use `print()`.

### Physics / pyfrc
- `physics.py` must live in the project root, not `sim/physics.py`.
- Use units from `pyfrc.physics.units import units`; do not construct standalone `pint.Quantity` objects.


---
