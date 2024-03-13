
#============================================================================
# tf2
#============================================================================

# pkg_name
set(pkg_name tf2)


# pkg_dependencies
set(pkg_dependencies stdboost ros_msgs)
set(pkg_dependencies_private  )


# pkg_src
set(pkg_src src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include ${PROJECT_SOURCE_DIR}/moveit_dependencies/log_helper/include/ )
target_include_directories(${pkg_name} PRIVATE ${PROJECT_SOURCE_DIR}/3rd/eigen3)


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

    MACRO(tf2_add_test test_name test_src)
        add_executable(${test_name} ${test_src})
        add_dependencies(${test_name} ${pkg_name})
        target_link_libraries(${test_name} ${pkg_name} gtest ros_helper)
        add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
    ENDMACRO()

    # test_tf2_cache_unittest
    tf2_add_test(test_tf2_cache_unittest test/cache_unittest.cpp)

    # test_tf2_static_cache_unittest
    tf2_add_test(test_tf2_static_cache_unittest test/static_cache_test.cpp)

    # test_tf2_simple_core
    tf2_add_test(test_tf2_simple_core test/simple_tf2_core.cpp)

    # test_tf2_speed
    tf2_add_test(test_tf2_speed test/speed_test.cpp)

    # test_tf2_transform_datatypes
    tf2_add_test(test_tf2_transform_datatypes test/test_transform_datatypes.cpp)
endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 3.0.2)
project(tf2)

find_package(console_bridge REQUIRED)
find_package(catkin REQUIRED COMPONENTS geometry_msgs rostime tf2_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES tf2
        DEPENDS console_bridge
        CATKIN_DEPENDS geometry_msgs tf2_msgs rostime)

include_directories(include ${catkin_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

# export user definitions

#CPP Libraries
add_library(tf2 src/cache.cpp src/buffer_core.cpp src/static_cache.cpp)
target_link_libraries(tf2 ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES})
add_dependencies(tf2 ${catkin_EXPORTED_TARGETS})

install(TARGETS tf2
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

# Tests
if (CATKIN_ENABLE_TESTING)

    catkin_add_gtest(test_cache_unittest test/cache_unittest.cpp)
    target_link_libraries(test_cache_unittest tf2 ${console_bridge_LIBRARIES})
    add_dependencies(test_cache_unittest ${catkin_EXPORTED_TARGETS})

    catkin_add_gtest(test_static_cache_unittest test/static_cache_test.cpp)
    target_link_libraries(test_static_cache_unittest tf2 ${console_bridge_LIBRARIES})
    add_dependencies(test_static_cache_unittest ${catkin_EXPORTED_TARGETS})

    catkin_add_gtest(test_simple test/simple_tf2_core.cpp)
    target_link_libraries(test_simple tf2 ${console_bridge_LIBRARIES})
    add_dependencies(test_simple ${catkin_EXPORTED_TARGETS})

    add_executable(speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
    target_link_libraries(speed_test tf2 ${console_bridge_LIBRARIES})
    add_dependencies(tests speed_test)
    add_dependencies(tests ${catkin_EXPORTED_TARGETS})

    catkin_add_gtest(test_transform_datatypes test/test_transform_datatypes.cpp)
    target_link_libraries(test_transform_datatypes tf2 ${console_bridge_LIBRARIES})
    add_dependencies(test_transform_datatypes ${catkin_EXPORTED_TARGETS})

endif ()
