# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_controller_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED controller_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(controller_FOUND FALSE)
  elseif(NOT controller_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(controller_FOUND FALSE)
  endif()
  return()
endif()
set(_controller_CONFIG_INCLUDED TRUE)

# output package information
if(NOT controller_FIND_QUIETLY)
  message(STATUS "Found controller: 0.0.0 (${controller_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'controller' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT controller_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(controller_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${controller_DIR}/${_extra}")
endforeach()
