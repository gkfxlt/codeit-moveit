
#============================================================================
# roscpp_traits
#============================================================================

# pkg_name
set(pkg_name roscpp_traits)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private)


# pkg_src
set(pkg_src )


# add_library
add_library(${pkg_name} INTERFACE)
target_include_directories(${pkg_name} INTERFACE include)


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


cmake_minimum_required(VERSION 2.8.3)
project(roscpp_traits)
#find_package(catkin REQUIRED)
#catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS cpp_common rostime)

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")
