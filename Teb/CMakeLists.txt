cmake_minimum_required(VERSION 3.8)
project(Teb)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system chrono)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
  set(ARCH_DIR "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  set(ARCH_DIR "x86_64")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(G2O_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH_DIR}")
set(G2O_LIBRARIES
  "${G2O_LIB_DIR}/libg2o_core.so"
  "${G2O_LIB_DIR}/libg2o_csparse_extension.so"
  "${G2O_LIB_DIR}/libg2o_ext_csparse.so"
  "${G2O_LIB_DIR}/libg2o_solver_csparse.so"
  "${G2O_LIB_DIR}/libg2o_stuff.so"
)
foreach(g2o_lib IN LISTS G2O_LIBRARIES)
  if(NOT EXISTS "${g2o_lib}")
    message(FATAL_ERROR "Required g2o runtime library not found: ${g2o_lib}")
  endif()
endforeach()

file(GLOB TEB_SRCS CONFIGURE_DEPENDS src/*.cpp)

add_library(Teb STATIC ${TEB_SRCS})
add_library(Teb::Teb ALIAS Teb)

target_include_directories(Teb
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/g2o>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/g2o_types>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/suitesparse>
    $<BUILD_INTERFACE:/usr/include/suitesparse>
    $<BUILD_INTERFACE:/usr/local/include/suitesparse>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:/usr/include/suitesparse>
)

target_link_libraries(Teb
  PUBLIC
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${G2O_LIBRARIES}
)

install(
  TARGETS Teb
  EXPORT TebTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

install(FILES
  "${G2O_LIB_DIR}/libg2o.so"
  ${G2O_LIBRARIES}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(TebTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost)

ament_package()
