
#============================================================================
# roscpp_serialization
#============================================================================

# pkg_name
set(pkg_name roscpp_serialization)


# pkg_dependencies
set(pkg_dependencies rostime roscpp_traits)
set(pkg_dependencies_private)


# pkg_src
set(pkg_src src/serialization.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)


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
#  add_executable(test_serialization test/test_serialization.cpp)
#  target_link_libraries(test_serialization
#          ${pkg_name}
#          gtest
#          )
endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 2.8.3)
project(roscpp_serialization)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

#find_package(catkin REQUIRED COMPONENTS cpp_common roscpp_traits rostime)
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roscpp_serialization
#  CATKIN_DEPENDS cpp_common roscpp_traits rostime
#)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(roscpp_serialization src/serialization.cpp)

#install(TARGETS roscpp_serialization
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")
