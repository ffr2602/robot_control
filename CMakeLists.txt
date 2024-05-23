cmake_minimum_required(VERSION 3.8)
project(robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

find_package(Armadillo REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(trajectory_tracking scripts/path_tracking.cpp)
ament_target_dependencies(trajectory_tracking rclcpp geometry_msgs sensor_msgs tf2 tf2_ros nav_msgs tf2_geometry_msgs)
target_link_libraries(trajectory_tracking ${ARMADILLO_LIBRARIES})

install(TARGETS
  trajectory_tracking
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/joy_control.py
  scripts/omni_controller.py
  scripts/path_tracking.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
