# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_claus_gazebo_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED claus_gazebo_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(claus_gazebo_FOUND FALSE)
  elseif(NOT claus_gazebo_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(claus_gazebo_FOUND FALSE)
  endif()
  return()
endif()
set(_claus_gazebo_CONFIG_INCLUDED TRUE)

# output package information
if(NOT claus_gazebo_FIND_QUIETLY)
  message(STATUS "Found claus_gazebo: 0.0.0 (${claus_gazebo_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'claus_gazebo' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT claus_gazebo_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(claus_gazebo_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${claus_gazebo_DIR}/${_extra}")
endforeach()
