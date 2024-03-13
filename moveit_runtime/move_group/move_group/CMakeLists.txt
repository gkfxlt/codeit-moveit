
#============================================================================
# move_group
#============================================================================

# pkg_name
set(pkg_name move_group)

add_definitions("-Wall -g")

# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private   tinyxml2 yaml-cpp move_group_capabilities_base moveit_plugin_loader fake_node_handle)


# pkg_src
set(pkg_src src/move_group.cpp src/param_loader.cpp)


# add_library
add_library(${pkg_name} SHARED ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
set_target_properties(${pkg_name} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
target_compile_definitions(${pkg_name} PRIVATE MOVE_GROUP_BUILDING_DLL)
if (UNIX)
    target_link_libraries(${pkg_name} pthread)
endif ()


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


# OUTPUT_DIRECTORY
set_target_properties(${pkg_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
set_target_properties(${pkg_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
set_target_properties(${pkg_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

# install
install(TARGETS ${pkg_name} DESTINATION ${INSTALL_LIB_PATH})
install(DIRECTORY moveit_config DESTINATION ${INSTALL_BIN_PATH})
install(DIRECTORY include DESTINATION ${INSTALL_ROOT_PATH})


## BUILD_TESTING
if (BUILD_TESTING)
    add_executable(test_move_group test/main.cpp)
    add_dependencies(test_move_group ${pkg_name})
    target_link_libraries(test_move_group ${pkg_name})

    install(TARGETS test_move_group DESTINATION ${INSTALL_TEST_PATH})
endif ()


# add_custom_command
add_custom_command(TARGET ${pkg_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/moveit_config ${PROJECT_BINARY_DIR}/bin/moveit_config
        COMMENT "Copy moveit_config to EXECUTABLE_OUTPUT_PATH"
        )

return()
############################################################################################


cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_move_group)

if (NOT "${CMAKE_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD 14)
endif ()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(Boost REQUIRED system filesystem date_time program_options thread)
find_package(catkin REQUIRED COMPONENTS
        moveit_core
        moveit_ros_planning
        actionlib
        roscpp
        pluginlib
        std_srvs
        tf2
        tf2_geometry_msgs
        tf2_ros
        )

catkin_package(
        LIBRARIES
        moveit_move_group_capabilities_base
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        actionlib
        moveit_core
        moveit_ros_planning
        roscpp
        std_srvs
        tf2_geometry_msgs
)

include_directories(include)
include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

add_library(moveit_move_group_capabilities_base
        src/move_group_context.cpp
        src/move_group_capability.cpp
        )
set_target_properties(moveit_move_group_capabilities_base PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(moveit_move_group_capabilities_base ${catkin_EXPORTED_TARGETS}) # wait until all *_msgs packages are finished being built

add_executable(move_group src/move_group.cpp)

add_executable(list_move_group_capabilities src/list_capabilities.cpp)

add_library(moveit_move_group_default_capabilities
        src/default_capabilities/move_action_capability.cpp
        src/default_capabilities/plan_service_capability.cpp
        src/default_capabilities/execute_trajectory_action_capability.cpp
        src/default_capabilities/query_planners_service_capability.cpp
        src/default_capabilities/kinematics_service_capability.cpp
        src/default_capabilities/state_validation_service_capability.cpp
        src/default_capabilities/cartesian_path_service_capability.cpp
        src/default_capabilities/get_planning_scene_service_capability.cpp
        src/default_capabilities/apply_planning_scene_service_capability.cpp
        src/default_capabilities/clear_octomap_service_capability.cpp
        src/default_capabilities/tf_publisher_capability.cpp
        )
set_target_properties(moveit_move_group_default_capabilities PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(moveit_move_group_default_capabilities ${catkin_EXPORTED_TARGETS})

target_link_libraries(moveit_move_group_capabilities_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(move_group moveit_move_group_capabilities_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(moveit_move_group_default_capabilities moveit_move_group_capabilities_base ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(list_move_group_capabilities ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS move_group list_move_group_capabilities
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS moveit_move_group_capabilities_base moveit_move_group_default_capabilities
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(PROGRAMS
        scripts/load_map scripts/save_map DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
        default_capabilities_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    # this test is flaky
    # add_rostest(test/test_cancel_before_plan_execution.test)
    add_rostest(test/test_check_state_validity_in_empty_scene.test)
endif ()
