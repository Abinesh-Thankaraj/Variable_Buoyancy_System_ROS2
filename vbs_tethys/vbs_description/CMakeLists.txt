cmake_minimum_required(VERSION 3.5)
project(vbs_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY 
    launch 
    meshes 
    urdf
    models
    meshes/materials
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ballast_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("hooks/resource_paths.dsv.in")
install(DIRECTORY launch meshes urdf DESTINATION share/${PROJECT_NAME})

ament_package()
