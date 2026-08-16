# Robot Architecture
Every FRC robot has the same overall architecture: **the Robot** and the **Driver Station Laptop**—connected by a wireless or wired network link.

**1. The Robot Side (On-board components)**

- **Sensors** and **Motors** are physically wired to the central controller.
- **roboRIO (The Brain)**: This is the main robot controller. It runs the compiled robot code, reads real-time data from the Sensors, processes that data through the code logic, and sends output commands to the Motors.
- **Radio (Wireless Access Point / Bridge)**: This device is directly connected to the roboRIO via an internal Ethernet cable. It acts as the robot's network gateway.

**2. The Communication Link**

- The Radio communicates with the outside world via a **Wi-Fi signal** (6GHz). As an alternative during setup or pit work, a physical **Ethernet cable** can be used instead of Wi-Fi.

**3. The Driver Station Side (Off-robot, in hands or pit area)**

- **Laptop Computer** running 3 primary pieces of softwares:
  - **Driver Station Software**: Sends driver joystick/gamepad commands to the robot and displays real-time telemetry (battery voltage, status, errors) received from the robot.
  - **Specific Toolkits**(e.g. REV Robotics' REV Hardware Client or CTRE Phoenix Tuner): debug or config specific hardware component.
  - **Development Environment (IDE / Simulation)**: Used by programmers to write, debug, and deploy new code to the roboRIO over the same network connection, as well as to run offline simulations.

---

**Data Flow Summary (How it works):**

- **Command Path:** Laptop → (Wi-Fi/Ethernet) → Radio → (internal Ethernet) → roboRIO → Motors.
- **Feedback Path:** Sensors → roboRIO → (internal Ethernet) → Radio → (Wi-Fi/Ethernet) → Laptop (Driver Station).
- **Deployment Path:** Laptop (IDE) → (Wi-Fi/Ethernet) → Radio → (internal Ethernet) → roboRIO (code overwritten and restarted).