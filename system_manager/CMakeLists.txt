cmake_minimum_required(VERSION 3.8)
project(system_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system chrono)
find_package(Teb REQUIRED)

add_executable(motion_plan src/manager.cpp)

target_include_directories(motion_plan
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(
  motion_plan
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(motion_plan
  Teb::Teb
  Eigen3::Eigen
  ${Boost_LIBRARIES}
)

set_target_properties(motion_plan PROPERTIES
  INSTALL_RPATH "$ORIGIN/../../../Teb/lib/Teb;$ORIGIN"
)

install(TARGETS
  motion_plan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
