
#============================================================================
# moveit_runtime
#============================================================================

add_subdirectory(moveit_runtime)
add_subdirectory(fake_node_handle)
add_subdirectory(plugin_loader)

add_subdirectory(tf2_ros)

add_subdirectory(constraint_sampler_manager_loader)

add_subdirectory(rdf_loader)

add_subdirectory(kinematics_plugin_loader)
add_subdirectory(robot_model_loader)

add_subdirectory(joint_state_publisher)
add_subdirectory(robot_state_publisher)

add_subdirectory(occupancy_map_monitor)
add_subdirectory(planning_scene_monitor)

add_subdirectory(trajectory_execution_manager)

add_subdirectory(planning_request_adapter_plugins)
add_subdirectory(planning_pipeline)

add_subdirectory(plan_execution)

add_subdirectory(pick_place)

add_subdirectory(move_group)


# plugins
add_subdirectory(controller_plugins)
add_subdirectory(kinematics_plugins)
add_subdirectory(planner_plugins)
add_subdirectory(perception)

