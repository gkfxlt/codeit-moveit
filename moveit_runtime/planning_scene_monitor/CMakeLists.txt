
#============================================================================
# moveit_planning_scene_monitor
#============================================================================

# pkg_name
set(pkg_name moveit_planning_scene_monitor)


# pkg_dependencies
set(pkg_dependencies moveit_planning_scene moveit_occupancy_map_monitor fake_node_handle tf2_ros)
set(pkg_dependencies_private   moveit_robot_model_loader joint_state_publisher robot_state_publisher)


# pkg_src
set(pkg_src
        src/planning_scene_monitor.cpp
        src/current_state_monitor.cpp
        src/trajectory_monitor.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
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


## BUILD_TESTING
if (BUILD_TESTING)
endif ()

return()
############################################################################################
