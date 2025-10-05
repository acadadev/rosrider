# ROSRider Cartographer

### 🤖 Cartographer Wrapper for ROSRider Platform

This package provides a simplified wrapper around the **Google Cartographer** library,
enabling easy and consistent execution of 2D SLAM (Simultaneous Localization and Mapping)
for the ROSRider platform in both simulated (Gazebo) and real-world robot environments.

![ROS2 Cartographer](https://docs.acada.dev/rosrider_doc/images/rosrider/rviz_cartographer.png)

The primary goal is to abstract the environment-specific setup differences,
allowing the user to switch between simulation and real hardware with simple launch arguments.

### 💡 What is Cartographer?
**Cartographer** is an open-source library for Simultaneous Localization and Mapping (SLAM), originally developed by Google.
It is highly regarded for its ability to generate high-resolution, globally consistent 2D and 3D maps in real-time across various
sensor configurations (LIDAR, IMU, odometry). Cartographer achieves this accuracy by using sophisticated techniques like
**scan matching** (Local SLAM) and **pose graph optimization with loop closure detection** (Global SLAM) to minimize accumulated
errors over long trajectories.  

For the `rosrider_cartographer` package, we specifically leverage its reliable 2D SLAM capabilities for mobile robot navigation.

### 🛠️ Prerequisites

Cartographer ROS: You must have the `cartographer_ros package` installed.

```commandline
sudo apt-get install ros-jazzy-cartographer-ros
```

### 🚀 Usage

This package uses a single Python launch file `cartographer.launch.py` to handle both real-robot and simulated scenarios by adjusting parameters.
For simulation set `use_sim_time:=True`.
The launch_rviz:=True argument is optional and automatically opens the visualization tool.

1. On the Real Robot (Live Mapping)

Use this command when running the package on your physical ROSRider robot.

```commandline
ros2 launch rosrider_cartographer cartographer.launch.py launch_rviz:=True
```

2. Using Gazebo Simulation

When using Gazebo, the `use_sim_time:=True` argument is mandatory. This synchronizes Cartographer's time clock with the simulation environment.

```commandline
ros2 launch rosrider_cartographer cartographer.launch.py use_sim_time:=True launch_rviz:=True
```

---
#### ACADA Robotics ● [https://acada.dev](https://acada.dev)  
[![ACADA Robotics](https://docs.acada.dev/rosrider_doc/images/logo.svg)](https://acada.dev)