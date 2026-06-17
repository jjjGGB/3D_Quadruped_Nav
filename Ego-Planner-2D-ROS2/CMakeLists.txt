cmake_minimum_required(VERSION 3.8)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -std=c++17 -g")
project(ego_planner)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(./planner/bspline_opt/include)
include_directories(./planner/path_searching/include)
include_directories(./planner/GridMap2D/include)
include_directories(./planner/plan_manage/include/plan_manage)
include_directories(./planner/plan_manage/include)
include_directories(./planner/include)

add_subdirectory(planner/bspline_opt)
add_subdirectory(planner/path_searching)
add_subdirectory(planner/GridMap2D)
add_subdirectory(planner/plan_manage)

target_include_directories(ego_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/bspline_opt/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/path_searching/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/GridMap2D/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/plan_manage/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/plan_manage/include/plan_manage>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/planner/include>
    $<INSTALL_INTERFACE:include>
)

# 编译节点
add_executable(motion_plan src/trajectory_publisher.cpp)
ament_target_dependencies(
  motion_plan
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
  visualization_msgs
)

add_executable(octo_path_optimizer_node src/octo_path_optimizer_node.cpp)
ament_target_dependencies(
  octo_path_optimizer_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

target_link_libraries(
  motion_plan
  bspline_opt  
  path_searching  
  local_map 
  ego_planner  
  ${Boost_LIBRARIES} 
)

target_link_libraries(
  octo_path_optimizer_node
  ego_planner
  bspline_opt
  path_searching
  local_map
  ${Boost_LIBRARIES}
)

# 安装节点
install(TARGETS
  motion_plan
  octo_path_optimizer_node
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ego_planner bspline_opt path_searching local_map
  EXPORT ego_plannerTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  planner/bspline_opt/include/
  planner/path_searching/include/
  planner/GridMap2D/include/
  planner/plan_manage/include/
  planner/include/
  DESTINATION include
)

ament_export_targets(ego_plannerTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost Eigen3)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
