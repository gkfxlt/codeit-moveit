
#============================================================================
# move_group_capabilities_base
#============================================================================

# pkg_name
set(pkg_name move_group_capabilities_default)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private   move_group_capabilities_base moveit_pick_place_planner)


# pkg_src
set(pkg_src
        src/move_action_capability.cpp
        src/plan_service_capability.cpp
        src/execute_trajectory_action_capability.cpp
        src/query_planners_service_capability.cpp
        src/kinematics_service_capability.cpp
        src/state_validation_service_capability.cpp
        src/cartesian_path_service_capability.cpp
        src/get_planning_scene_service_capability.cpp
        src/apply_planning_scene_service_capability.cpp
        src/clear_octomap_service_capability.cpp
        src/tf_publisher_capability.cpp
        src/pick_place_action_capability.cpp)


# add_library
add_library(${pkg_name} SHARED ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
set_target_properties(${pkg_name} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
target_compile_definitions(${pkg_name} PRIVATE CAPABILITIES_DEFAULT_BUILDING_DLL)

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


# PLUGINS_OUTPUT_PATH
set_target_properties(${pkg_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})
set_target_properties(${pkg_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})
set_target_properties(${pkg_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})

# install
install(TARGETS ${pkg_name} DESTINATION ${INSTALL_PLUGINS_PATH})


## BUILD_TESTING
if (BUILD_TESTING)
endif ()

return()
############################################################################################
