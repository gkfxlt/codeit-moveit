
#============================================================================
# moveit_planning_scene
#============================================================================

# pkg_name
set(pkg_name moveit_planning_scene)


# pkg_dependencies
set(pkg_dependencies moveit_kinematic_constraints moveit_trajectory_processing)
set(pkg_dependencies_private  )


# pkg_src
set(pkg_src src/planning_scene.cpp)


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

	# MACRO
	MACRO(planning_scene_add_test test_name test_src)
		add_executable(${test_name} ${test_src})
		add_dependencies(${test_name} ${pkg_name})
		target_link_libraries(${test_name} ${pkg_name} gtest moveit_test_utils)
		add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
	ENDMACRO()

	# test_planning_scene
	planning_scene_add_test(test_planning_scene test/test_planning_scene.cpp)

	# test_multi_threaded
	planning_scene_add_test(test_multi_threaded test/test_multi_threaded.cpp)

endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_planning_scene)

add_library(${MOVEIT_LIB_NAME} src/planning_scene.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model
  moveit_robot_state
  moveit_exceptions
  moveit_transforms
  moveit_collision_detection_fcl
  moveit_kinematic_constraints
  moveit_robot_trajectory
  moveit_trajectory_processing
  ${LIBOCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${MOVEIT_LIB_NAME}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_planning_scene test/test_planning_scene.cpp)
  target_link_libraries(test_planning_scene ${MOVEIT_LIB_NAME})

  catkin_add_gtest(test_multi_threaded test/test_multi_threaded.cpp)
  target_link_libraries(test_multi_threaded moveit_planning_scene moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES})
endif()
