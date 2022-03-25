cmake_minimum_required(VERSION 3.5)
project(storz_straight_exoscope_hsi_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY gazebo meshes urdf
    DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}.dsv")

ament_package()
