# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_champ_bringup_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED champ_bringup_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(champ_bringup_FOUND FALSE)
  elseif(NOT champ_bringup_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(champ_bringup_FOUND FALSE)
  endif()
  return()
endif()
set(_champ_bringup_CONFIG_INCLUDED TRUE)

# output package information
if(NOT champ_bringup_FIND_QUIETLY)
  message(STATUS "Found champ_bringup: 0.1.0 (${champ_bringup_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'champ_bringup' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${champ_bringup_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(champ_bringup_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${champ_bringup_DIR}/${_extra}")
endforeach()
