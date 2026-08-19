# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_abb_irb140_description_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED abb_irb140_description_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(abb_irb140_description_FOUND FALSE)
  elseif(NOT abb_irb140_description_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(abb_irb140_description_FOUND FALSE)
  endif()
  return()
endif()
set(_abb_irb140_description_CONFIG_INCLUDED TRUE)

# output package information
if(NOT abb_irb140_description_FIND_QUIETLY)
  message(STATUS "Found abb_irb140_description: 1.0.0 (${abb_irb140_description_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'abb_irb140_description' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT abb_irb140_description_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(abb_irb140_description_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${abb_irb140_description_DIR}/${_extra}")
endforeach()
