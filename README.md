### ROSRider Control Card Driver for ROS2

The **ROSRider board** is a dedicated control unit designed to operate within the ROS 2 ecosystem,
acting as the primary interface between the robot's motors and sensors and the higher-level ROS 2 nodes.
Its driver, located in the [rosrider_node](https://github.com/acadadev/rosrider/tree/main/rosrider_node) package, handles the essential communication protocols,
abstracting the hardware complexity away from the main application logic.  

**This setup transforms the ROSRider board into a plug-and-play platform for robotics development.**

Furthermore, the board's behavior and hardware configuration are parametric, with all settings easily managed and configured via a **YAML file**.

| Package                                                                                       | Description               |
|-----------------------------------------------------------------------------------------------|---------------------------|
| [rosrider_interfaces](https://github.com/acadadev/rosrider/tree/main/rosrider_interfaces)     | Interfaces                |
| [rosrider_node](https://github.com/acadadev/rosrider/tree/main/rosrider_node)                 | Hardware Driver           |
| [rosrider_description](https://github.com/acadadev/rosrider/tree/main/rosrider_description)   | URDF Description          |
| [rosrider_nav](https://github.com/acadadev/rosrider/tree/main/rosrider_nav)                   | Navigation                |
| [rosrider_cartographer](https://github.com/acadadev/rosrider/tree/main/rosrider_cartographer) | Cartographer Mapping      |
| [rosrider_image](https://github.com/acadadev/rosrider/tree/main/rosrider_image)               | Image Processing Pipeline |


ACADA Robotics <info@acada.dev>
