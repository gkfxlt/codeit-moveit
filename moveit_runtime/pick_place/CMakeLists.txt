
#============================================================================
# moveit_pick_place_planner
#============================================================================

# pkg_name
set(pkg_name moveit_pick_place_planner)


# pkg_dependencies
set(pkg_dependencies moveit_constraint_sampler_manager_loader moveit_planning_pipeline moveit_plan_execution)
set(pkg_dependencies_private  )


# pkg_src
set(pkg_src
        src/pick_place_params.cpp
        src/manipulation_pipeline.cpp
        src/reachable_valid_pose_filter.cpp
        src/approach_and_translate_stage.cpp
        src/plan_stage.cpp
        src/pick_place.cpp
        src/pick.cpp
        src/place.cpp)


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
endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_pick_place_planner)

add_library(${MOVEIT_LIB_NAME}
  src/pick_place_params.cpp
  src/manipulation_pipeline.cpp
  src/reachable_valid_pose_filter.cpp
  src/approach_and_translate_stage.cpp
  src/plan_stage.cpp
  src/pick_place.cpp
  src/pick.cpp
  src/place.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
