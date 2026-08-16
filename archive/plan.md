# RobotPy 测试环境配置计划

> 目标：在今年暑假完成 FRC Python（RobotPy）本地测试环境搭建，验证从 Java 切换到 Python 的可行性，产出可运行的最小示例和仿真流程文档。
> 时间：暑假期间；先完成本机环境，再评估是否引入 CI。

\---

## 1\. 环境准备

### 1.1 检查 Python 版本

* 安装 Python 3.12（与 2026 赛季 roboRIO 部署版本对齐；RobotPy 2026.2.x 支持 3.10–3.13，3.12 是稳妥选择）。
* 确认 `python --version` 与 `python -m pip --version` 正常。

### 1.2 创建隔离虚拟环境

* 在项目根目录 `D:\\Program\\Hardware\\try-robotpy` 下创建 `.venv`。
* 命令：

```powershell
  python -m venv .venv
  .venv\\Scripts\\activate
  python -m pip install --upgrade pip
  ```

* 后续所有 pip 安装都在虚拟环境中进行。

\---

## 2\. 安装 RobotPy 核心工具链

### 2.1 安装 RobotPy

```powershell
python -m pip install robotpy==2026.2.2
```

### 2.2 安装仿真与测试依赖

```powershell
python -m pip install pyfrc robotpy-halsim-gui robotpy-halsim-ds-socket
```

### 2.3 安装 Vendor 依赖（匹配俱乐部现有硬件）

根据 report.md 中的版本对照，安装以下包：

```powershell
python -m pip install robotpy-rev==2026.0.4
python -m pip install phoenix6==26.3.0
python -m pip install robotpy-navx==2026.0.1.1
python -m pip install robotpy-apriltag robotpy-commands-v2
```

> 如果俱乐部明年实际硬件有变化，在这里替换或增减 vendor 包。

\---

## 3\. 初始化最小 RobotPy 项目

### 3.1 创建项目骨架

在项目根目录生成以下文件：

```
try-robotpy/
├── robot.py              # 机器人入口
├── pyproject.toml        # 依赖与 RobotPy 配置
├── tests/
│   ├── \_\_init\_\_.py
│   └── test\_basic.py     # 基础单元测试
└── sim/
    └── physics.py        # 仿真物理引擎（可选第一步先留空模板）
```

### 3.2 编写 `pyproject.toml`

* 声明项目元数据。
* 在 `\[tool.robotpy]` 下列出所有需要同步到 roboRIO 的 vendor 包。
* 示例模板：

```toml
  \[build-system]
  requires = \["robotpy\~=2026.2"]

  \[tool.robotpy]
  robotpy\_version = "2026.2.2"
  robotpy\_extras = \["apriltag", "commands2"]

  \[tool.robotpy.requires]
  robotpy-rev = "==2026.0.4"
  phoenix6 = "==26.3.0"
  robotpy-navx = "==2026.0.1.1"
  ```

### 3.3 编写最小 `robot.py`

* 基于 `commands-v2` 的 TimedCommandRobot 模板。
* 至少包含：

  * `robotInit`
  * `teleopInit` / `teleopPeriodic`
  * `autonomousInit` / `autonomousPeriodic`
  * `testInit` / `testPeriodic`
  * `disabledInit` / `disabledPeriodic`
* 初始示例只打印生命周期日志，不驱动真实硬件。

\---

## 4\. 同步与校验依赖

### 4.1 运行 `robotpy sync`

```powershell
python -m robotpy sync
```

* 验证 pyproject.toml 中的 vendor 包能被正确解析。
* 检查本地缓存的 roboRIO 部署包是否下载成功。

### 4.2 验证安装

```powershell
python -c "import robotpy; print(robotpy.\_\_version\_\_)"
python -c "import rev, phoenix6, navx; print('vendors ok')"
python -m robotpy --help
```

\---

## 5\. 仿真测试

### 5.1 无 GUI 仿真

```powershell
python robot.py sim --nogui
```

* 确认机器人生命周期正常进入 disabled/teleop/autonomous/test。
* 观察日志无异常退出。

### 5.2 GUI 仿真

```powershell
python robot.py sim --gui
```

* 确认 WPILib Simulation GUI 能启动。
* 检查 2D Field 小部件、电机/传感器面板是否加载。

### 5.3 Driver Station Socket 模式（可选）

```powershell
python robot.py sim --ds-socket
```

* 如果本机有 FRC Driver Station，验证能否连接仿真机器人。

### 5.4 添加简单物理模型

* 在 `sim/physics.py` 中实现 `PhysicsEngine`。
* 示例：一个两轮差动底盘的运动学模型，接收 PWM 速度，更新位姿。
* 在 GUI 中观察机器人是否随控制输入移动。

\---

## 6\. 单元测试

### 6.1 编写基础测试

* 使用 `pyfrc.test\_support` 的 `setup\_robot` fixture。
* 测试机器人初始化不抛异常。
* 测试一个简单命令能否被调度并结束。

### 6.2 运行测试

```powershell
python robot.py test
# 或
pytest tests/
```

* 目标：全部通过，且能在无 GUI 环境下运行。

### 6.3 评估 CI 可行性（暂时不用）

* 在 GitHub Actions 或本地 runner 上运行 `pytest`。
* 确认仿真依赖（`robotpy-halsim-gui` 等）在 headless 环境下可跳过 GUI。

\---

## 7\. 记录与验收标准

### 7.1 输出文档

* 新增 `setup.md`，记录：

  * 实际安装的 Python 版本和全部包版本（`pip freeze` 输出）。
  * 遇到的坑和解决方案。
  * 仿真启动命令和常用调试方法。

### 7.2 验收标准

* \[ ] `python -m venv .venv` 成功，虚拟环境可激活。
* \[ ] `python -m pip install robotpy==2026.2.2` 成功。
* \[ ] `python -m robotpy sync` 无错误。
* \[ ] `python robot.py sim --nogui` 能跑满 30 秒不崩溃。
* \[ ] `python robot.py sim --gui` 能启动仿真 GUI。
* \[ ] `python robot.py test` 至少通过 1 个基础测试。
* \[ ] 最小示例代码被其他队员克隆后，按文档能复现同样结果。

\---

## 8\. 下一步（开学后）

* 根据今年机器人代码，挑选一个子系统做 Python 移植练习。
* 评估 Command-based 框架与现有 Java 代码的结构映射。
* 关注 SystemCore 2027 动态，适时创建 `2027-alpha` 分支测试 RobotPy 2027.x。

\---

## 备注

* 所有命令默认在 PowerShell 中执行；如使用 cmd 或 Git Bash，激活虚拟环境的路径需调整。
* 不直接修改系统 Python，全部依赖走 `.venv`。
* 若某 vendor 包安装失败，先单独 `pip install` 该包排查，再写进 `pyproject.toml`。

