cmake_minimum_required(VERSION 3.5)
project(rosrider_node)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosrider_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(rosrider_node src/rosrider.cpp src/i2c_hal.c)
ament_target_dependencies(rosrider_node rosrider_interfaces rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2_ros)

install(TARGETS rosrider_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch param DESTINATION share/${PROJECT_NAME})

ament_package()