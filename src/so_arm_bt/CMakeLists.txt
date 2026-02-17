cmake_minimum_required(VERSION 3.8)
project(so_arm_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(so_arm_bt_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)


add_executable(bt_executor_node src/bt_executor_node.cpp)
ament_target_dependencies(bt_executor_node
  rclcpp
  behaviortree_cpp_v3
  moveit_ros_planning_interface
  geometry_msgs
  so_arm_bt_interfaces
  std_srvs
  moveit_msgs
)


target_include_directories(bt_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# Install executable
install(TARGETS bt_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/vision_processor_node.py
  scripts/bt_compiler_node.py
  DESTINATION lib/${PROJECT_NAME}
)




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

ament_package()
