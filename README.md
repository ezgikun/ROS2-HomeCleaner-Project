# 🧹 ROS2 Autonomous Home Cleaner Robot

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Docker](https://img.shields.io/badge/Container-Docker-2496ED)
![Language](https://img.shields.io/badge/Language-Python-yellow)
![Status](https://img.shields.io/badge/Status-Completed-success)

## 📖 Project Overview
This project implements an **autonomous vacuum cleaning robot** simulation using **ROS2 Humble** on Ubuntu 22.04. The system is designed to map an unknown home environment using **SLAM**,
navigate safely using the **Nav2 Stack**, and perform a full coverage cleaning task using a custom **Python-based Finite State Machine (FSM)**.

The entire development environment is **Dockerized**, allowing the project to run on any machine with a **single command** without complex dependency installation.

### 📺 Demo Video
**[CLICK HERE TO WATCH THE PROJECT DEMO VIDEO] https://youtu.be/2TWdJ3Nkwd4**

### Project Video
**[CLICK HERE TO WATCH THE PROJECT DEMO VIDEO] https://youtu.be/2TWdJ3Nkwd4**

---

## ✨ Key Features
* **🐋 Dockerized Environment:** Full ROS2 desktop, Gazebo, and RViz encapsulated in a single container.
* **🗺️ SLAM Integration:** Used **SLAM Toolbox** (Async Mode) to generate a high-precision occupancy grid map (`.pgm` & `.yaml`).
* **📍 Autonomous Navigation:** Implemented **Nav2 Stack** (AMCL + DWB Controller) for path planning and dynamic obstacle avoidance.
* **🧠 Custom Coverage Planner:** A Python script (`cleaner_manager.py`) using **Nav2 Simple Commander API** to manage cleaning states (Cleaning, Returning, Emergency Stop).
* **⚡ One-Liner Execution:** Automated `launch` system that starts Gazebo, Nav2, RViz, and the Cleaning Agent sequentially.

---

## 🚀 Quick Start (For Instructors & Reviewers)

To run this project on your local machine, follow these steps.

### Prerequisites
* Docker installed.
* NVIDIA Drivers (for GPU acceleration) & NVIDIA Container Toolkit (recommended).

### 1. Clone the Repository
```bash
git clone [https://github.com/ezgikun/ROS2-HomeCleaner-Project.git](https://github.com/ezgikun/ROS2-HomeCleaner-Project.git)
cd ROS2-HomeCleaner-Project
```
2. Build the Docker Image

Build the environment using the provided Dockerfile. This ensures all ROS2 packages (Nav2, SLAM, etc.) are installed.
```bash
docker build -t final_project_image .
```
## 🕹️ Usage Scenarios

You can run the project in different modes depending on your needs. Below are the commands for the most common scenarios.

### 1. 🏆 Full Autonomous Demo (The "One-Liner")
This is the main execution command. It launches the simulation, loads the map, starts the navigation stack, and automatically triggers the **Cleaning Manager** to clean the house autonomously.

```bash
xhost +local:root && docker run -it --net=host --ipc=host --pid=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/dri:/dev/dri \
    --rm final_project_image \
    bash -c "source /opt/ros/humble/setup.bash && source /root/homecleaner_ws/install/setup.bash && cd /root/homecleaner_ws && ros2 launch homcleaner_bot start_all.launch.py"
```
### 2. Creating a New Map (SLAM Mode)

If you want to map a new environment, follow these two steps using two separate terminals.

Step 1: Start Simulation & SLAM (Terminal 1) This launches Gazebo and the SLAM Toolbox in async mode.
```bash
xhost +local:root && docker run -it --net=host --ipc=host --pid=host
--env="DISPLAY=$DISPLAY"     --env="QT_X11_NO_MITSHM=1"     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
--device /dev/dri:/dev/dri     --rm final_project_image
bash -c "source /opt/ros/humble/setup.bash && source /root/homecleaner_ws/install/setup.bash && ros2 launch homecleaner_bot simulation.launch.py & sleep 5 && ros2 launch slam_toolbox online_async_launch.py"

```
Step 2: Start Teleoperation (Terminal 2) Open a new terminal to control the robot manually using WASD keys.
Bash
```bash
docker exec -it $(docker ps -q -l) bash -c "source /root/homecleaner_ws/install/setup.bash && python3 /root/homecleaner_ws/src/homecleaner_bot/wasd_control.py"
```
After mapping, use the RViz MapSaver plugin or CLI to save your map.

### 3. 📍 Navigation Testing (Manual Goal)

Use this command to test the Nav2 stack without running the autonomous cleaning script. You can set goals manually using the "2D Goal Pose" tool in RViz.
```bash
xhost +local:root && docker run -it --net=host --ipc=host --pid=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device /dev/dri:/dev/dri \
    --rm final_project_image \
    bash -c "source /opt/ros/humble/setup.bash && source /root/homecleaner_ws/install/setup.bash && ros2 launch homecleaner_bot simulation.launch.py & sleep 5
    && ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/root/homecleaner_ws/src/homecleaner_bot/maps/my_home_map.yaml autostart:=True & sleep 5
    && ros2 run rviz2 rviz2 -d /root/homecleaner_ws/src/homecleaner_bot/my_nav.rviz"
```

📂 Project Structure
```text
.
├── Dockerfile                      # Builds the complete ROS2 & Gazebo environment
└── src
    └── homecleaner_bot             # Main ROS2 package directory
        ├── package.xml             # Defines package dependencies and metadata
        ├── setup.py                # Python build configuration and installation rules
        ├── my_nav.rviz             # Custom RViz configuration for visualization
        ├── cleaner_manager.py      # MAIN NODE: Autonomous Coverage Planner & FSM Logic
        ├── wasd_control.py         # Tool: Manual Teleoperation script (used during mapping)
        ├── launch/                 # System startup and orchestration scripts
        │   ├── simulation.launch.py  # Launches Gazebo & Spawns the Robot Model
        │   ├── navigation.launch.py  # Launches Nav2 Stack (AMCL + Planner + Controller)
        │   └── start_all.launch.py   # FINAL ONE-LINER: Orchestrates Gazebo, Nav2, RViz, and Cleaning
        ├── maps/                   # Map data generated by SLAM Toolbox
        │   ├── my_home_map.pgm       # Visual Occupancy Grid Map (Image)
        │   └── my_home_map.yaml      # Map Metadata (Resolution, Origin, Thresholds)
        ├── urdf/                   # Robot Physical Description
        │   └── robot.urdf.xacro      # TurtleBot3 Waffle Pi Xacro definition
        └── world/                  # Simulation Environment Assets
            ├── my_home.world         # The custom home environment file for Gazebo
            └── my_home_layout/       # Custom 3D Model assets for the house
                ├── model.config
                └── model.sdf
```
🛠️ Tech Stack

    Framework: ROS 2 Humble 

    Simulation: Gazebo Classic

    Visualization: RViz2

    Languages: Python 3, XML (Launch/URDF)

    Tools: Docker, Git, SLAM Toolbox, Navigation2
