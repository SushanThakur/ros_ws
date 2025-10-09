cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)  # Added for ROS 2 components
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)  # Added for Trigger service

# Add the joy_servo component as a shared library
add_library(joy_servo SHARED
  src/joy_servo.cpp
)
target_include_directories(joy_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(joy_servo PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(joy_servo
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  control_msgs
  std_srvs
  rclcpp
  rclcpp_components
)

# Register the component
rclcpp_components_register_node(joy_servo
  PLUGIN "moveit_servo::JoyToServoPub"
  EXECUTABLE joy_servo_node
)

# Other executables
add_executable(hello_moveit src/hello_moveit.cpp)
target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(hello_moveit
  moveit_ros_planning_interface
  rclcpp
)

add_executable(ps_mover src/ps_mover.cpp)
ament_target_dependencies(ps_mover
  moveit_ros_planning_interface
  rclcpp
  geometry_msgs
)

add_executable(ps_sub src/ps_sub.cpp)
ament_target_dependencies(ps_sub
  moveit_ros_planning_interface
  geometry_msgs
)

# Install targets
install(TARGETS hello_moveit ps_mover ps_sub joy_servo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(geometry_msgs REQUIRED)
  # Skip linter checks for copyrights and cpplint
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()