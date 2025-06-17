cmake_minimum_required(VERSION 3.8)
project(my_robot_arm)

# Default to C++14
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros_ign_gazebo REQUIRED)
find_package(ros_ign_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(gz_ros2_control REQUIRED)

#find_package(joint_state_publisher REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# If you have C++ executables, declare them here:
# add_executable(your_node src/your_node.cpp)
# ament_target_dependencies(your_node rclcpp urdf rviz2 robot_state_publisher joint_state_publisher)
# install(TARGETS your_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf launch config worlds
DESTINATION share/${PROJECT_NAME}/
)
install(PROGRAMS
scripts/manual_joint_input.py
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
scripts/move_object.py
DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
scripts/object_detect.py
DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  controller_manager
  hardware_interface
  urdf
  rviz2
  robot_state_publisher
)


ament_package()
