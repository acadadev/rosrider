### ROSRider Control Card Driver for ROS2

The **ROSRider board** is a dedicated control unit designed to operate within the ROS 2 ecosystem,
acting as the primary interface between the robot's motors and sensors and the higher-level ROS 2 nodes.
Its driver, located in the [rosrider_node](https://github.com/acadadev/rosrider/tree/main/rosrider_node) package, handles the essential communication protocols,
abstracting the hardware complexity away from the main application logic.  

**This setup transforms the ROSRider board into a plug-and-play platform for robotics development.**

Furthermore, the board's behavior and hardware configuration are parametric, with all settings easily managed and configured via a **YAML file**.

| Package                                                                                       | Description                                                                                                                                                 |
|-----------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [rosrider_node](https://github.com/acadadev/rosrider/tree/main/rosrider_node)                 | This is the primary ROS 2 driver that manages low-level communication with the ROSRider control board and exposes its functionality to the ROS 2 graph.     |
| [rosrider_description](https://github.com/acadadev/rosrider/tree/main/rosrider_description)   | Contains the Universal Robot Description Format (URDF) file, defining the robot's physical structure, joints, and sensors for visualization and simulation. |
| [rosrider_interfaces](https://github.com/acadadev/rosrider/tree/main/rosrider_interfaces)     | Defines the custom ROS 2 messages and services used for communication between the hardware driver and other robot components.                               |                             
| [rosrider_nav](https://github.com/acadadev/rosrider/tree/main/rosrider_nav)                   | Implements the high-level navigation stack, enabling the robot to perform path planning, obstacle avoidance, and goal following.                            |
| [rosrider_cartographer](https://github.com/acadadev/rosrider/tree/main/rosrider_cartographer) | Integrates the Cartographer library for high-fidelity SLAM (Simultaneous Localization and Mapping), allowing the robot to build and localize within a map.  |
| [rosrider_image](https://github.com/acadadev/rosrider/tree/main/rosrider_image)               | Handles the image processing pipeline, managing data from cameras and performing necessary operations like filtering or feature extraction.                 |


ACADA Robotics <info@acada.dev>
