# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_aquabot_ekf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED aquabot_ekf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(aquabot_ekf_FOUND FALSE)
  elseif(NOT aquabot_ekf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(aquabot_ekf_FOUND FALSE)
  endif()
  return()
endif()
set(_aquabot_ekf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT aquabot_ekf_FIND_QUIETLY)
  message(STATUS "Found aquabot_ekf: 0.1.0 (${aquabot_ekf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'aquabot_ekf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT aquabot_ekf_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(aquabot_ekf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${aquabot_ekf_DIR}/${_extra}")
endforeach()
