## ROS2 Navigation Commands and Workflow

This guide outlines the common ROS 2 launch commands for essential navigation tasks, including Simultaneous Localization and Mapping (SLAM), localization with a known map, and configuring keepout filters.

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

![Gazebo Simulation Willow World](https://raw.githubusercontent.com/acadadev/rosrider_doc/refs/heads/main/images/rosrider/gazebo_willow_world.png)

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


#### TODO

1. add pictures