cmake_minimum_required(VERSION 3.22)
project(so_arm100_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  control
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/meshes
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES "SO-ARM100/URDF/SO_5DOF_ARM100_8j_URDF.SLDASM/meshes/Moving Jaw.STL"
        DESTINATION share/${PROJECT_NAME}/meshes
        RENAME Moving_Jaw.STL)

ament_package()
