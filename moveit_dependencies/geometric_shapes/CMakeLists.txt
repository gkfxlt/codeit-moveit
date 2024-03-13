
#============================================================================
# geometric_shapes
#============================================================================

# pkg_name
set(pkg_name geometric_shapes)


# pkg_dependencies
set(pkg_dependencies ros_msgs)
set(pkg_dependencies_private   stdboost assimp qhull octomap)


# pkg_src
set(pkg_src
        src/aabb.cpp
        src/bodies.cpp
        src/body_operations.cpp
        src/mesh_operations.cpp
        src/shape_extents.cpp
        src/shape_operations.cpp
        src/shape_to_marker.cpp
        src/shapes.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include ${PROJECT_SOURCE_DIR}/3rd/eigen3 ${PROJECT_SOURCE_DIR}/moveit_dependencies/log_helper/include )


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

    # MACRO
    MACRO(geometric_shapes_add_test test_name test_src)
        add_executable(${test_name} ${test_src})
        add_dependencies(${test_name} ${pkg_name})
        target_link_libraries(${test_name} ${pkg_name} gtest)
        target_compile_definitions(${test_name} PRIVATE TEST_RESOURCES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/resources")
        add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/resources)
    ENDMACRO()

    # test_basics
    geometric_shapes_add_test(test_basics test/test_basics.cpp)

    # test_point_inclusion
    geometric_shapes_add_test(test_point_inclusion test/test_point_inclusion.cpp)

    # test_bounding_sphere
    geometric_shapes_add_test(test_bounding_sphere test/test_bounding_sphere.cpp)

    # test_bounding_box
    geometric_shapes_add_test(test_bounding_box test/test_bounding_box.cpp)

    # test_bounding_cylinder
    geometric_shapes_add_test(test_bounding_cylinder test/test_bounding_cylinder.cpp)

    # test_create_mesh
    geometric_shapes_add_test(test_create_mesh test/test_create_mesh.cpp)

    # test_loaded_meshes
    geometric_shapes_add_test(test_loaded_meshes test/test_loaded_meshes.cpp)

    # test_shapes
    geometric_shapes_add_test(test_shapes test/test_shapes.cpp)

    # test_ray_intersection
    geometric_shapes_add_test(test_ray_intersection test/test_ray_intersection.cpp)

    # test_body_operations
    geometric_shapes_add_test(test_body_operations test/test_body_operations.cpp)
endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 3.0.2)
project(geometric_shapes)

add_compile_options(-std=c++11)

# Set compile options
set(PROJECT_COMPILE_OPTIONS
        -Wall
        #  -Wextra
        #  -Wwrite-strings
        #  -Wunreachable-code
        #  -Wpointer-arith
        #  -Wredundant-decls
        #  -Wno-unused-parameter
        )

#list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

#if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
#  message("${PROJECT_NAME}: You did not request a specific build type: Choosing 'Release' for maximum performance")
#  set(CMAKE_BUILD_TYPE Release)
#endif()

# TODO(henningkayser): Remove policy fix when assimp 5.1 is available
# Suppress policy warning in assimp (https://github.com/assimp/assimp/pull/2722)
#set(CMAKE_POLICY_DEFAULT_CMP0012 NEW)
#find_package(ASSIMP QUIET)
#if(NOT ASSIMP_FOUND)
#  find_package(PkgConfig REQUIRED)
#  # assimp is required, so REQUIRE the second attempt
#  pkg_check_modules(ASSIMP_PC REQUIRED assimp)
#  set(ASSIMP_INCLUDE_DIRS ${ASSIMP_PC_INCLUDE_DIRS})
#endif()

# find *absolute* paths to ASSIMP_LIBRARIES
# Both, pkg-config and assimp's cmake-config don't provide an absolute library path.
# For, pkg-config the path is in ASSIMP_PC_LIBRARY_DIRS, for cmake in ASSIMP_LIBRARY_DIRS.
#find_library(ASSIMP_ABS_LIBRARIES NAMES ${ASSIMP_LIBRARIES} assimp HINTS ${ASSIMP_LIBRARY_DIRS} ${ASSIMP_PC_LIBRARY_DIRS})
#set(ASSIMP_LIBRARIES "${ASSIMP_ABS_LIBRARIES}")

#find_package(Boost REQUIRED system filesystem)

#find_package(console_bridge REQUIRED)

#find_package(Eigen3 REQUIRED)

#find_package(octomap REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  eigen_stl_containers
#  random_numbers
#  resource_retriever
#  shape_msgs
#  visualization_msgs
#)

#catkin_package(
#  INCLUDE_DIRS
#    include
#  LIBRARIES ${PROJECT_NAME} ${OCTOMAP_LIBRARIES}
#  CATKIN_DEPENDS
#    eigen_stl_containers
#    random_numbers
#    shape_msgs
#    visualization_msgs
#  DEPENDS
#    EIGEN3
#    OCTOMAP
#    console_bridge
#  )

#find_package(QHULL REQUIRED)

include_directories(include)
#include_directories(SYSTEM
#  ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${ASSIMP_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS}
#  ${QHULL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
        src/aabb.cpp
        src/bodies.cpp
        src/body_operations.cpp
        src/mesh_operations.cpp
        src/shape_extents.cpp
        src/shape_operations.cpp
        src/shape_to_marker.cpp
        src/shapes.cpp
        )
target_compile_options(${PROJECT_NAME} PRIVATE ${PROJECT_COMPILE_OPTIONS})
#set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})

target_link_libraries(${PROJECT_NAME} assimp-vc141-mtd qhullcpp ${ASSIMP_LIBRARIES} ${QHULL_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
    # Unit tests
    add_subdirectory(test)
endif ()

#install(TARGETS ${PROJECT_NAME}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
