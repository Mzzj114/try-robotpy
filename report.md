# 🔬 研究成果总结

1️⃣ RobotPy (FRC Python) 环境搭建

状态：成熟可用 ✅

Python 自 2024 年起已是 FRC 官方支持语言

2026 赛季支持 Python 3.10/3.11/3.12/3.13/3.14，roboRIO 上仅支持 3.14

安装极其简单：

python3 -m pip install robotpy

项目用 pyproject.toml 管理依赖，用 robotpy sync 同步 vendor 包

pyfrc 提供单元测试和实时机器人模拟

官方文档已整合进 WPILib 文档站

2️⃣ 各大 Vendor 兼容性

| Vendor | PyPI 包 | 说明 |
|--------|---------|------|
| REV Robotics (SPARK MAX/Flex) | robotpy-rev ✅ | 支持 NEO/NEO 550，完整配置类（SparkMaxConfig），闭环控制 |
| CTR Electronics (Talon FX, Falcon 500) | robotpy-ctre (Phoenix 5) / phoenix6 (官方) ✅ | Phoenix 6 有官方 Python 发行版，含 swerve + PathPlanner 示例 |
| NavX (Kauai Labs) | robotpy-navx ✅ | 最新 2026.2，I2C/USB 都支持 |
| Limelight | 社区包 Robotpy-Limelight ⚠️ | Limelight 走 NetworkTables（标准 FRC 机制），无需特定包也能直接用 NT 读取 tx/ty/ta 等数据 |

所有主要硬件的 Python 绑定都存在且活跃维护。

3️⃣ SystemCore — 2027 年取代 roboRIO

确认属实 ✅ 不是谣言。

SystemCore：全新实时微控制器，2027 赛季起取代 roboRIO

规格：四核 ARM Cortex‑A76，板载 IMU、LED、显示屏

接口：CAN ×2、I2C、USB、Ethernet、可重构 I/O（PWM/DIO/Analog）

已有 Alpha/Beta 测试计划，部分团队已收到测试机

WPILib 2027 文档分支已上线，部分 PR 已合并移除 roboRIO 特有代码

Python 支持：RobotPy 已有 robotpy-native-wpilib 2027.0.0a5 在 PyPI 上 → Python 在 SystemCore 上也走得通

roboRIO 是否同时支持？ 未官方确认，但 2027 WPILib 分支正逐步移除 roboRIO 特定代码

配套还有 MotionCore（FTC 用）

📋 结论

✅ FRC Python 完全可行。所有关键硬件都有 Python 支持。建议从官方文档开始：
https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html

# 📦 版本对照

roboRIO (当前)

pip install robotpy → 2026.2.2

robotpy-rev==2026.0.4

robotpy-ctre / phoenix6==26.3.0

robotpy-navx==2026.0.1.1

robotpy-apriltag / commands-v2 均有稳定版

SystemCore (2027)

pip install robotpy~=2027.0.0a6 → 当前 Alpha 6

robotpy-rev ✅ 已有 2027.0.0a4

robotpy-ctre / phoenix6 ❌ 暂无

robotpy-navx ❌ 暂无

robotpy_extras 已移除 → vendor 包改在 requires 里独立写

预计 10 月 Kit of Parts 分发时会有正式版。

# 🔬 FRC Python 仿真研究报告

1. 如何用 RobotPy 做 simulation？

安装依赖：

pip install pyfrc robotpy-halsim-gui

运行仿真：

标准 GUI 仿真：python3 robot.py sim

带 WPILib 原生仿真 GUI：python3 robot.py sim --gui

无 GUI 纯命令行：python3 robot.py sim --nogui

DS Socket 模式（可接实体 Driver Station）：python3 robot.py sim --ds-socket

pyfrc 内置 physics engine 机制，在 sim/ 目录下放 physics.py，定义 PhysicsEngine 类，通过 update_sim(now, tm_diff) 在每一轮仿真循环中更新电机和传感器状态。

2. Simulation 能力评估

能模拟的：

✅ 底盘运动（TwoMotor/Mecanum/FourMotor drivetrain 模型）

✅ 电机 PWM 信号 → 速度/位置计算

✅ 编码器反馈（通过 drivetrain 计算 wheel speeds 反推）

✅ 限位开关（可按时间/位置条件触发）

✅ 陀螺仪（通过 self.physics_controller.get_pose() 获得朝向角）

✅ 2D 场地显示（WPILib Simulation GUI 内建 field widget）

✅ 网络表格（NetworkTables）通信

✅ SmartDashboard/Shuffleboard/Glass 仪表板（连接 localhost）

✅ Unit test 集成（pytest + pyfrc.test 框架）

✅ REVLib Spark MAX / CTRE Phoenix 设备的 SimState 接口

不能模拟的：

❌ 真实摄像头视觉处理（无实际图像输入，除非自制模拟器）

❌ 物理碰撞/投掷物物理（pyfrc physics 只做 2D 运动学）

❌ 电流/功率消耗

❌ 真实传感器噪声模式（需要自己写模型）

❌ 网络延迟/掉包

3. 常见仿真工具

| 工具 | 用途 |
|------|------|
| pyfrc | 核心仿真框架，提供 PhysicsEngine、pytest 测试集成、上传器 |
| robotpy-halsim-gui | WPILib 原生仿真 GUI，含 2D 场地、电机/Sensor 数据面板 |
| robotpy-halsim-ds-socket | 仿真 DS Socket，允许实体 Driver Station 连接仿真 |
| robotpy-halsim-ws | WebSocket 仿真接口（--ws-server / --ws-client），用于远程查看 |
| pytest + pyfrc | 纯代码级仿真测试，无 GUI 快速验证逻辑 |
| REVLib / CTRE SimState | 第三方设备的模拟状态接口 |
| WPILib Glass | 实时数据可视化工具 |
| AdvantageScope | 替代性数据可视化/分析 |

4. 无硬件情况下能做什么程度的测试？

完全可以做：

✅ 底盘控制逻辑：编写 PhysicsEngine 模拟底盘运动，验证 pid 控制、自动路径规划

✅ 自动程序（Auto）测试：完整的 autonomous 阶段仿真，验证轨迹跟踪

✅ 命令调度：Command-based 框架的调度序列验证

✅ 状态机：判断条件 → 切换状态的逻辑

✅ 传感器数据处理：模拟编码器读数，测试 closed-loop 控制

✅ 仪表板通信：SmartDashboard/Shuffleboard 数据显示验证

✅ 单元测试：用 pytest 做 CI 集成，每次提交自动运行

物理效果限制：

电机特性（惯性、摩擦力）需要手动建模，pyfrc 不自动模拟

视觉依赖需要自己写模拟数据注入

真实比赛中的突发情况（接线松动、掉包、电池电压下降）不模拟

总结：90% 的逻辑层代码可以在仿真中充分测试。缺少硬件时，pyfrc + halsim-gui 是最高效的离线开发方案，尤其适合自动程序调试和控制算法迭代。