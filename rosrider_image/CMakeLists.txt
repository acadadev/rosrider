cmake_minimum_required(VERSION 3.5)
project(rosrider_image)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/apriltag_overlay.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fix_p_matrix_rect.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
