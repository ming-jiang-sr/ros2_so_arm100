cmake_minimum_required(VERSION 3.22)
project(so_arm100_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install package directories
install(DIRECTORY urdf control rviz launch DESTINATION share/${PROJECT_NAME}/)

# Install SO-ARM101-Assets meshes
install(
  DIRECTORY SO-ARM101-Assets/
  DESTINATION share/${PROJECT_NAME}/SO-ARM101-Assets
  FILES_MATCHING
  PATTERN "*.stl"
  PATTERN "*.STL"
  PATTERN "*.dae"
  PATTERN "*.DAE"
  PATTERN "*.obj"
  PATTERN "*.OBJ"
  PATTERN "*.mtl"
  PATTERN "*.MTL"
)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION
                             lib/${PROJECT_NAME})

ament_package()
