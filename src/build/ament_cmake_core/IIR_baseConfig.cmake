# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_IIR_base_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED IIR_base_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(IIR_base_FOUND FALSE)
  elseif(NOT IIR_base_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(IIR_base_FOUND FALSE)
  endif()
  return()
endif()
set(_IIR_base_CONFIG_INCLUDED TRUE)

# output package information
if(NOT IIR_base_FIND_QUIETLY)
  message(STATUS "Found IIR_base: 0.0.0 (${IIR_base_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'IIR_base' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT IIR_base_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(IIR_base_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${IIR_base_DIR}/${_extra}")
endforeach()
