## Robot Description

This package provides the **complete kinematic and visual description** of the ROSRider mobile robot.
It contains the **URDF** model, **mesh files**, and launch files for viewing the robot in **RVIZ** and simulation in **Gazebo.**
The key directories are: `urdf/` (URDF files), `meshes/` (STL/DAE files), and `launch/` (Launch files for visualization).

```commandline
ros2 launch rosrider_description urdf.launch.py
```

The `joint_state_publisher` node included in the launch file allows you to manually manipulate the robot's
joint positions using a slider GUI in RViz, which is essential for **verifying the kinematic and visual integrity**
of the URDF model before proceeding to simulation.

![Robot URDF Visualization](https://docs.acada.dev/rosrider_doc/images/rosrider/rosrider_description.png)

