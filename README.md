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
```

## 🌐 2. Launching the ROS2 Simulation Environment
Execute the following commands across separate, individual terminal windows. Do not close them while running the assignments.

🔹 Terminal 1: Spin up the Gazebo Simulation World
Initialize the environment variables and launch the default TurtleBot3 world map.

```text
Bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

🔹 Terminal 2: Launch the Nav2 Stack with Environment Map
This loads the Navigation2 stack and feeds it your pre-saved map coordinates.

```
Bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map_world.yaml
```
⚠️ CRITICAL STEP: Once RViz2 boots up completely via Terminal 2, you must localize the robot manually:

Click the 2D Pose Estimate button on the top RViz2 toolbar.

Click and drag green directional arrows directly onto the map position that corresponds to where the TurtleBot is physically sitting inside the Gazebo environment window.

Verify that the localized laser scan lines closely match the physical walls on your map matrix to properly align the coordinate transformations (/map ➔ /odom ➔ /base_footprint).

💻 3. Running the MATLAB Controller
Open your MATLAB environment and follow these configuration steps to spin up the reactive node.

Step A: Set Workspace Directory
Point your current working directory to the folder containing your control scripts:

# Matlab
```
cd ~/matlab_ws
```
Step B: Compile Action Interfaces (First-time Setup Only)
If you have not compiled the custom Navigation2 actions yet, run the message generator in the command window. Point the path string to your specific custom_msgs folder directory:

#Matlab
````
ros2genmsg('/path/to/your/matlab_workspace/custom_msgs')
````
Step C: Execute the Controller Core
Execute the master controller containing your background subscriber callbacks and parallel FSM loops:

#Matlab
````
run('main.m')
````
🛑 Safe Termination
To cleanly shut down the ROS2 nodes and halt motor outputs at any time:

Click directly inside the active MATLAB Command Window.

Press Ctrl + C on your keyboard to terminate the running script safely.
