# ROSRider Cartographer

### 🤖 Cartographer Wrapper for ROSRider Platform

This package provides a simplified wrapper around the **Google Cartographer** library,
enabling easy and consistent execution of 2D SLAM (Simultaneous Localization and Mapping)
for the ROSRider platform in both simulated (Gazebo) and real-world robot environments.

![ROS2 Cartographer](https://docs.acada.dev/rosrider_doc/images/rosrider/rviz_cartographer.png)

The primary goal is to abstract the environment-specific setup differences,
allowing the user to switch between simulation and real hardware with simple launch arguments.

---

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

---

### 🚀 Usage

This package uses a single Python launch file `cartographer.launch.py` to handle both real-robot and simulated scenarios by adjusting parameters.
The `launch_rviz:=True` argument RVIZ tool, initialized with the appropriate settings for the Cartographer output.

> ⚠️ **Important Note on Navigation:**
>  This package is only responsible for Mapping with **Cartographer.** To enable goal-driven movement, you must separately launch the **navigation stack.**

#### On the Real Robot (Live Mapping)

Use this command when running the package on your physical ROSRider robot.

```commandline
ros2 launch rosrider_cartographer cartographer.launch.py launch_rviz:=True
```

#### Using Gazebo Simulation

When using Gazebo, set the `use_sim_time:=True` argument.
You can find all the simulation assets and launch files inside the [rosrider_gz](https://github.com/acadadev/rosrider_gz) repository.

```commandline
ros2 launch rosrider_cartographer cartographer.launch.py use_sim_time:=True
```

---

### Documentation

For complete and comprehensive guides on all aspects of the ROSRider project, please refer to the dedicated documentation site: [https://docs.acada.dev/rosrider_doc](https://docs.acada.dev/rosrider_doc)

---
#### ACADA Robotics ● [https://acada.dev](https://acada.dev)  
[![ACADA Robotics](https://docs.acada.dev/rosrider_doc/images/logo.svg)](https://acada.dev)