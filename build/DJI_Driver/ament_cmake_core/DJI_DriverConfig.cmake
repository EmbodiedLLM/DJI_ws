# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_DJI_Driver_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED DJI_Driver_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(DJI_Driver_FOUND FALSE)
  elseif(NOT DJI_Driver_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(DJI_Driver_FOUND FALSE)
  endif()
  return()
endif()
set(_DJI_Driver_CONFIG_INCLUDED TRUE)

# output package information
if(NOT DJI_Driver_FIND_QUIETLY)
  message(STATUS "Found DJI_Driver: 0.0.1 (${DJI_Driver_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'DJI_Driver' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${DJI_Driver_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(DJI_Driver_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${DJI_Driver_DIR}/${_extra}")
endforeach()
