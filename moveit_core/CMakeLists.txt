
#============================================================================
# moveit_core
#============================================================================

add_subdirectory(macros)
add_subdirectory(profiler)
add_subdirectory(exceptions)
add_subdirectory(transforms)

add_subdirectory(kinematics_base)
add_subdirectory(kinematic_constraints)
add_subdirectory(constraint_samplers)  # for pick_place and ompl_planner

add_subdirectory(robot_model)
add_subdirectory(robot_state)
add_subdirectory(robot_trajectory)
add_subdirectory(trajectory_processing)

add_subdirectory(utils)

add_subdirectory(collision_detection)
add_subdirectory(collision_detection_fcl)
add_subdirectory(planning_scene)

add_subdirectory(planning_interface)
add_subdirectory(planning_request_adapter)

add_subdirectory(controller_manager)

return()
