# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_octo_planner3d_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED octo_planner3d_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(octo_planner3d_FOUND FALSE)
  elseif(NOT octo_planner3d_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(octo_planner3d_FOUND FALSE)
  endif()
  return()
endif()
set(_octo_planner3d_CONFIG_INCLUDED TRUE)

# output package information
if(NOT octo_planner3d_FIND_QUIETLY)
  message(STATUS "Found octo_planner3d: 0.1.0 (${octo_planner3d_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'octo_planner3d' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${octo_planner3d_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(octo_planner3d_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${octo_planner3d_DIR}/${_extra}")
endforeach()
