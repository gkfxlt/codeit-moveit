
#============================================================================
# octomath
#============================================================================

# pkg_name
set(pkg_name octomath)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private)


# pkg_src
set(pkg_src Vector3.cpp Quaternion.cpp Pose6D.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC)


# add_dependencies
if (DEFINED pkg_dependencies)
  add_dependencies(${pkg_name} ${pkg_dependencies})
  target_link_libraries(${pkg_name} ${pkg_dependencies})
  target_add_interfaces(${pkg_name} ${pkg_dependencies})
endif ()

if (DEFINED pkg_dependencies_private)
  add_dependencies(${pkg_name} ${pkg_dependencies_private})
  target_link_libraries(${pkg_name} ${pkg_dependencies_private})
  target_add_interfaces_private(${pkg_name} ${pkg_dependencies_private})
endif ()


## BUILD_TESTING
if (BUILD_TESTING)
endif ()

return()
############################################################################################


SET (octomath_SRCS
  Vector3.cpp
  Quaternion.cpp
  Pose6D.cpp
 )


#ADD_LIBRARY( octomath SHARED ${octomath_SRCS})
ADD_LIBRARY( octomath ${octomath_SRCS})

#SET_TARGET_PROPERTIES( octomath PROPERTIES
#  VERSION ${OCTOMAP_VERSION}
#  SOVERSION ${OCTOMAP_SOVERSION}
#  INSTALL_NAME_DIR ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}   # this seems to be necessary for MacOS X
#)
# INSTALL_NAME_DIR seems to be necessary for MacOS X

ADD_LIBRARY( octomath-static STATIC ${octomath_SRCS})
SET_TARGET_PROPERTIES(octomath-static PROPERTIES OUTPUT_NAME "octomath")

if(NOT EXISTS "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
  file(MAKE_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap")
endif()

export(TARGETS octomath APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-targets.cmake")

#export(TARGETS octomath octomath-static
#  APPEND FILE "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/cmake/octomap/octomap-targets.cmake")

#install(TARGETS octomath octomath-static
#  EXPORT octomap-targets
#  INCLUDES DESTINATION include
#  ${INSTALL_TARGETS_DEFAULT_ARGS}
#)
