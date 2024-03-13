
#============================================================================
# moveit_default_planning_request_adapter_plugins
#============================================================================

# pkg_name
set(pkg_name moveit_default_planning_request_adapter_plugins)


# pkg_dependencies
set(pkg_dependencies )
set(pkg_dependencies_private   moveit_planning_request_adapter moveit_plugin_loader fake_node_handle moveit_trajectory_processing)


# pkg_src
set(pkg_src
        src/empty.cpp
        src/fix_start_state_bounds.cpp
        src/fix_start_state_collision.cpp
        src/fix_start_state_path_constraints.cpp
        src/fix_workspace_bounds.cpp
        src/add_time_parameterization.cpp
        src/add_iterative_spline_parameterization.cpp
        src/add_time_optimal_parameterization.cpp
        src/resolve_constraint_frames.cpp)


# add_library
add_library(${pkg_name} SHARED ${pkg_src})
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


set(MOVEIT_LIB_NAME moveit_default_planning_request_adapter_plugins)

set(SOURCE_FILES
        src/empty.cpp
        src/fix_start_state_bounds.cpp
        src/fix_start_state_collision.cpp
        src/fix_start_state_path_constraints.cpp
        src/fix_workspace_bounds.cpp
        src/add_time_parameterization.cpp
        src/add_iterative_spline_parameterization.cpp
        src/add_time_optimal_parameterization.cpp
        src/resolve_constraint_frames.cpp
        )

add_library(${MOVEIT_LIB_NAME} ${SOURCE_FILES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_list_request_adapter_plugins src/list.cpp)
target_link_libraries(moveit_list_request_adapter_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS moveit_list_request_adapter_plugins
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
