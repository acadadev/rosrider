### ROSRider Control Card Driver for ROS2

The **ROSRider board** is a dedicated control unit designed to operate within the ROS 2 ecosystem,
acting as the primary interface between the robot's motors and sensors and the higher-level ROS 2 nodes.
Its driver, located in the `rosrider_node` package, handles the essential communication protocols,
abstracting the hardware complexity away from the main application logic.  

**This setup transforms the ROSRider board into a plug-and-play platform for robotics development.**

Furthermore, the board's behavior and hardware configuration are parametric, with all settings easily managed and configured via a **YAML file**.

#### Hardware Interface
   
[rosrider_interfaces](https://github.com/acadadev/rosrider/tree/main/rosrider_interfaces)  

#### ROS Driver
[rosrider_node](https://github.com/acadadev/rosrider/tree/main/rosrider_node)  

#### Robot Description
[rosrider_description](https://github.com/acadadev/rosrider/tree/main/rosrider_description) 

#### Navigation 
[rosrider_nav](https://github.com/acadadev/rosrider/tree/main/rosrider_nav)  

#### Cartographer
[rosrider_cartographer](https://github.com/acadadev/rosrider/tree/main/rosrider_cartographer)  


#### Imaging Pipeline
[rosrider_image](https://github.com/acadadev/rosrider/tree/main/rosrider_image)  


ACADA Robotics <info@acada.dev>
