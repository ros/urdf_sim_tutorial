cmake_minimum_required(VERSION 3.5)
project(urdf_sim_tutorial)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
