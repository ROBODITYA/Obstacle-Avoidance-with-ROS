# TurtleBot3 Navigation & Reactive Obstacle Avoidance with ROS2 and MATLAB

This repository contains a comprehensive implementation of autonomous mobile robot control using **MATLAB** integrated with **ROS2 (Humble)**. The project is split into two primary paradigms: deliberative navigation using the ROS2 Navigation Stack (Nav2) via Action Clients, and a localized, real-time reactive obstacle avoidance system driven by parallel Finite State Machines (FSM).



---

## 🛠 Features

*   **Dead Reckoning & Odometry Control:** Open-loop and closed-loop scripts (`forward.m`, `turn.m`) tracking position via `$[x, y, \theta]$` coordinate frames.
*   **Nav2 Action Client:** Asynchronous waypoint navigation using `nav2_msgs/msg/NavigateToPose` with active feedback tracking and goal preemption.
*   **Square Path Execution:** Automated sequential execution of multi-point waypoint matrices using synchronous `sendGoalAndWait` patterns.
*   **Reactive FSM Obstacle Avoidance:** Direct sensor-to-actuator control loop utilizing parallel State Machines for decoupled velocity scheduling and directional steering adjustments.
*   **Memory-Lock States:** High-sensitivity laser scan scaling coupled with dedicated `completeLeftTurn`/`completeRightTurn` states to prevent localized robot oscillations in tight corners.

---

## 📂 Repository Structure

```text
~/matlab_ws/
├── main.m                  # Entry point: ROS2 initialization and main FSM control loop
├── ScanHandle.m            # Handle class reference passing for concurrent scan data
├── scanCallback.m          # Background callback processing raw LiDAR arrays to rmin/phimin
├── OdometryMsg2Pose.m      # Quaternion-to-Yaw transformation helper
└── README.md               # Setup and execution documentation
