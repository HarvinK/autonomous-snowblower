cmake_minimum_required(VERSION 3.0.2)
project(asb_description)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  urdf
  xacro
)

catkin_package()

install(
  DIRECTORY launch meshes urdf rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
