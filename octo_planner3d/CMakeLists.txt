cmake_minimum_required(VERSION 3.16)
project(octo_planner3d LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# PCL 的 io 组件会通过 VTK 引入 VTK::mpi；在较新的 CMake/VTK 组合中，
# VTK-targets.cmake 期望 MPI::MPI_C 已经存在。先显式查找 MPI，避免
# 单独编译 OctoPlanner 时出现 “VTK::mpi contains MPI::MPI_C but target was not found”。
find_package(MPI REQUIRED COMPONENTS C CXX)
find_package(PCL REQUIRED COMPONENTS io common)
find_package(ament_cmake QUIET)

add_library(octomap SHARED IMPORTED GLOBAL)
set_target_properties(octomap PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomap.so.1.10"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/octomap/include"
)

add_library(octomath SHARED IMPORTED GLOBAL)
set_target_properties(octomath PROPERTIES
  IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomath.so.1.10"
  INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/octomap/include"
)

add_subdirectory(octomap)
add_subdirectory(planner)

add_executable(globalPlan main.cpp)
target_link_libraries(globalPlan PRIVATE GlobalPlanner Octomap)

if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(visualization_msgs REQUIRED)

  add_executable(octo_planner_rviz_node src/octo_planner_rviz_node.cpp)
  target_link_libraries(octo_planner_rviz_node GlobalPlanner Octomap)
  set_target_properties(octo_planner_rviz_node PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    INSTALL_RPATH "$ORIGIN"
  )
  target_compile_definitions(
    octo_planner_rviz_node
    PRIVATE OCTO_PLANNER3D_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  ament_target_dependencies(
    octo_planner_rviz_node
    rclcpp
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    tf2_ros
    visualization_msgs
  )

  install(TARGETS
    globalPlan
    octo_planner_rviz_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomap.so.1.10"
    "${CMAKE_CURRENT_SOURCE_DIR}/lib/liboctomath.so.1.10"
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY launch rviz config
    DESTINATION share/${PROJECT_NAME}
  )
  install(DIRECTORY octomap/pcd_files
    DESTINATION share/${PROJECT_NAME}/octomap
  )
  ament_package()
endif()
