cmake_minimum_required(VERSION 3.5)
project(robotic-arm-urdf-min)

find_package(ament_cmake REQUIRED)

ament_package()

# Install folders like config, launch, meshes, urdf
install(DIRECTORY config launch meshes urdf launch rviz worlds
  DESTINATION share/${PROJECT_NAME}
)
