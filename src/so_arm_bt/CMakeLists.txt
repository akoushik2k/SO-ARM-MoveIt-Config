cmake_minimum_required(VERSION 3.8)
project(so_arm_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(so_arm_bt_interfaces REQUIRED)

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


include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_executable(bt_executor_node src/bt_executor_node.cpp)
ament_target_dependencies(bt_executor_node
  rclcpp
  rclcpp_action
  behaviortree_cpp
  moveit_ros_planning_interface
  geometry_msgs
  control_msgs
  sensor_msgs
  vision_msgs
  so_arm_bt_interfaces
  std_srvs
)

add_executable(vision_processor_node 
  src/vision_processor_node.cpp
  src/object_detector.cpp
  src/grasp_pose_estimator.cpp
)
ament_target_dependencies(vision_processor_node
  rclcpp
  sensor_msgs
  vision_msgs
  cv_bridge
  image_transport
  tf2_ros
  tf2_geometry_msgs
  so_arm_bt_interfaces
)

# Explicitly link system libraries to avoid conda conflicts
find_library(CURL_LIB curl PATHS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)
find_library(TIFF_LIB tiff PATHS /usr/lib/x86_64-linux-gnu NO_DEFAULT_PATH)

target_link_libraries(vision_processor_node 
  ${OpenCV_LIBS}
  ${CURL_LIB}
  ${TIFF_LIB}
)

# Fix library conflicts with conda
set_target_properties(bt_executor_node PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH FALSE
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu"
  BUILD_RPATH "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu"
)
set_target_properties(vision_processor_node PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH FALSE
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu"
  BUILD_RPATH "/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu"
)

# Install executables
install(TARGETS 
  bt_executor_node
  vision_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch behavior_trees config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
