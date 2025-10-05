cmake_minimum_required(VERSION 3.22)
project(moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

add_executable(moveit_config src/moveit_config.cpp)
ament_target_dependencies(
  moveit_config
  "moveit_ros_planning_interface"
  "rclcpp"
  "geometry_msgs"
  "moveit_ros_planning_interface"
  "geometry_msgs"
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
