### ROSRider Control Card Driver for ROS2

The **ROSRider board** is a dedicated control unit designed to operate within the ROS 2 ecosystem,
acting as the primary interface between the robot's motors and sensors and the higher-level ROS 2 nodes.
Its driver, located in the [rosrider_node](https://github.com/acadadev/rosrider/tree/main/rosrider_node) package, handles the essential communication protocols,
abstracting the hardware complexity away from the main application logic.  

**This setup transforms the ROSRider board into a plug-and-play platform for robotics development.**

Furthermore, the board's behavior and hardware configuration are parametric, with all settings easily managed and configured via a **YAML file**.

![ROSRider Control Board](https://docs.acada.dev/rosrider_doc/images/rosrider_system.png)

---

### Software Packages

| Package                                                                                       | Description                                                                                                                                                 |
|-----------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [rosrider_node](https://github.com/acadadev/rosrider/tree/main/rosrider_node)                 | This is the primary ROS 2 driver that manages low-level communication with the ROSRider control board and exposes its functionality to the ROS 2 graph.     |
| [rosrider_description](https://github.com/acadadev/rosrider/tree/main/rosrider_description)   | Contains the Universal Robot Description Format (URDF) file, defining the robot's physical structure, joints, and sensors for visualization and simulation. |
| [rosrider_interfaces](https://github.com/acadadev/rosrider/tree/main/rosrider_interfaces)     | Defines the custom ROS 2 messages and services used for communication between the hardware driver and other robot components.                               |                             
| [rosrider_nav](https://github.com/acadadev/rosrider/tree/main/rosrider_nav)                   | Implements the high-level navigation stack, enabling the robot to perform path planning, obstacle avoidance, and goal following.                            |
| [rosrider_cartographer](https://github.com/acadadev/rosrider/tree/main/rosrider_cartographer) | Integrates the Cartographer library for high-fidelity SLAM (Simultaneous Localization and Mapping), allowing the robot to build and localize within a map.  |
| [rosrider_image](https://github.com/acadadev/rosrider/tree/main/rosrider_image)               | Handles the image processing pipeline, managing data from cameras and performing necessary operations like filtering or feature extraction.                 |

### Simulations

To facilitate development, testing, and algorithmic validation without needing physical hardware, the ROSRider project includes the `rosrider_gz` package.
This component provides the necessary configuration files, robot models, and launch procedures to run a high-fidelity Gazebo (Gazebo Sim) simulation environment.
By integrating the ROSRider robot and its environment within the simulator, developers can fully test the `rosrider_nav`, and `rosrider_cartographer`
packages in a virtual world.  

This allows for rapid iteration and safe debugging of control, navigation, and mapping algorithms before deployment onto the actual robot,
serving as your virtual sandbox for all things ROSRider. You can find all the simulation assets and launch files inside the [rosrider_gz](https://github.com/acadadev/rosrider_gz) repository.  

---

### Documentation

For complete and comprehensive guides on all aspects of the ROSRider project, please refer to the dedicated documentation site: [https://docs.acada.dev/rosrider_doc](https://docs.acada.dev/rosrider_doc)

---

ACADA Robotics <info@acada.dev>
