# 🧭 ROS2 Navigation Commands and Workflow

This guide outlines the common ROS 2 launch commands for essential navigation tasks, including Simultaneous Localization and Mapping (SLAM), localization with a known map, and configuring keepout filters.

![ROS2 Navigation Simulation](https://docs.acada.dev/rosrider_doc/images/rosrider/navigation_with_keepout.png)

*Fig 1. RVIZ visualization program displaying navigation within known map*

### Simultaneous Localization and Mapping (SLAM)

SLAM is used to **create a map** of an environment while simultaneously tracking the robot's position within that map. Use this when the environment is unknown.

| Task       | Environment | Command                                                                    |
|:-----------|:------------|----------------------------------------------------------------------------|
| SLAM       | Real Robot  | ```ros2 launch rosrider_nav bringup_slam2.launch.py use_sim_time:=False``` |
| Navigation | Real Robot  | ```ros2 launch rosrider_nav bringup_nav2.launch.py use_sim_time:=False```  |

### Saving Maps

After navigating the robot, use the following command to save the map and generate the required ```map_name.yaml``` and ```map_name.pgm``` files.

```commandline
ros2 run nav2_map_server map_saver_cli -f map_name
```

Move the files to the `rosrider_nav/map` folder.
This allows you to use the `map:=map_file_path` argument when running localization with your own maps. 

### Localization

| Task         | Environment | Command                                                                           |
|:-------------|:------------|-----------------------------------------------------------------------------------|
| Localization | Real Robot  | ```ros2 launch rosrider_nav bringup_localization.launch.py use_sim_time:=False``` |
| Navigation   | Real Robot  | ```ros2 launch rosrider_nav bringup_nav2.launch.py use_sim_time:=False```         |

### Keepout Filters

Keepout filters define areas on the map that the robot must avoid (e.g., stairs, sensitive equipment). **Localization** must be active before you can use the keepout filters.

| Task           | Environment | Command                                                                      |
|:---------------|:------------|------------------------------------------------------------------------------|
| Keepout Filter | Real Robot  | ```ros2 launch rosrider_nav bringup_keepout.launch.py use_sim_time:=False``` |

**Note:** The **nav_params.yaml** file must be configured to support keepout filter functionality.

```commandline
local_costmap:
  local_costmap:
    ros__parameters:
      ...	
      plugins: ["voxel_layer", "inflation_layer"]
      ...
      filters: ["keepout_filter"]
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: False
        filter_info_topic: "/costmap_filter_info"
      ...
global_costmap:
  global_costmap:
    ros__parameters:
      ...
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      ...
      filters: ["keepout_filter"]
      ...
      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: False
        filter_info_topic: "/costmap_filter_info"

```

### Visualization

Use RViz to visualize the robot, sensor data, map, and navigation planning outputs.

```commandline
ros2 launch nav2_bringup rviz_launch.py
```

### Use with Gazebo Simulator

To run navigation tasks in the Gazebo simulation environment, you must first launch the simulator and then execute the SLAM or Localization commands with the use_sim_time argument set to True.

#### Launch Gazebo Simulator

```commandline
ros2 launch rosrider_gz_bringup world_willow.launch.py launch_rviz:=False
```

![Gazebo Simulation Willow World](https://docs.acada.dev/rosrider_doc/images/rosrider/gazebo_caretta.png)

*Fig 2. Gazebo Simulation with Willow Garage Environment.*

#### Execute Navigation in Simulation: 

Append ```use_sim_time:=True``` to the relevant command from sections on SLAM and Localization

**For SLAM**

| Task       | Environment | Command                                                                   |
|:-----------|:------------|---------------------------------------------------------------------------|
| SLAM       | Simulation  | ```ros2 launch rosrider_nav bringup_slam2.launch.py use_sim_time:=True``` |
| Navigation | Simulation  | ```ros2 launch rosrider_nav bringup_nav2.launch.py use_sim_time:=True```  |

**For Localization**

| Task         | Environment | Command                                                                          |
|:-------------|:------------|----------------------------------------------------------------------------------|
| Localization | Simulation  | ```ros2 launch rosrider_nav bringup_localization.launch.py use_sim_time:=True``` |
| Navigation   | Simulation  | ```ros2 launch rosrider_nav bringup_nav2.launch.py use_sim_time:=True```         |

**For Keepout Filters**

| Task           | Environment | Command                                                                     |
|:---------------|:------------|-----------------------------------------------------------------------------|
| Keepout Filter | Simulation  | ```ros2 launch rosrider_nav bringup_keepout.launch.py use_sim_time:=True``` |

### Supplementary Visualization and Context

The **global costmap** provides a high-level, static view of the entire environment to plan long-distance paths, while the **local costmap** is smaller, dynamic, 
and continuously updated with new sensor data for immediate obstacle avoidance and finer path adjustments.

The robot's primary navigation functions, including the **global planner** for long-range pathfinding and the **local controller** for immediate obstacle avoidance, rely heavily on the distinct information provided by these two costmaps.

![RVIZ Navigation Local Map](https://docs.acada.dev/rosrider_doc/images/rosrider/nav_local_global_map.png)

*Fig 3. RVIZ interface visualizing robot data while navigating*

The RVIZ screen provides a crucial visualization of the robot's localization within the known map and allows the user to command the robot by sending goal points via
the **2D Goal Pose** tool. Once a goal is established, the Nav2 stack processes the costmap data to generate a safe global path and dynamically manages the local plan
in real-time, ensuring the robot avoids both known static features and newly detected moving obstacles along the way.

![RVIZ Navigation with Keepout](https://docs.acada.dev/rosrider_doc/images/rosrider/rviz_navigation_with_keepout.png)

*Fig 4. RVIZ interface showing the whole localization map*

### Documentation

For complete and comprehensive guides on all aspects of the ROSRider project, please refer to the dedicated documentation site: [https://docs.acada.dev/rosrider_doc](https://docs.acada.dev/rosrider_doc)

---
#### ACADA Robotics ● [https://acada.dev](https://acada.dev)  
[![ACADA Robotics](https://docs.acada.dev/rosrider_doc/images/logo.svg)](https://acada.dev)