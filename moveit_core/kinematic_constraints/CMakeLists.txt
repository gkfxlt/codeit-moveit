
#============================================================================
# moveit_kinematic_constraints
#============================================================================

# pkg_name
set(pkg_name moveit_kinematic_constraints)


# pkg_dependencies
set(pkg_dependencies moveit_kinematics_base moveit_collision_detection_fcl moveit_utils)
set(pkg_dependencies_private   tf2)


# pkg_src
set(pkg_src
		src/kinematic_constraint.cpp
		src/eigen_conversions/eigen_msg.cpp
		src/utils.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include ../../moveit_dependencies/log_helper/include/)


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
	# test_constraints
	add_executable(test_constraints test/test_constraints.cpp)
	target_include_directories(test_constraints PRIVATE ${pkg_include_public} ${tf2_interface})
	target_link_libraries(test_constraints
			${pkg_name}
			moveit_test_utils
			gtest
			)
	add_test(NAME test_constraints COMMAND test_constraints WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

return()
############################################################################################



set(MOVEIT_LIB_NAME moveit_kinematic_constraints)

add_library(${MOVEIT_LIB_NAME}
  src/kinematic_constraint.cpp
  src/utils.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
  moveit_robot_model moveit_kinematics_base moveit_robot_state moveit_collision_detection_fcl moveit_utils
  ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${MOVEIT_LIB_NAME}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_constraints test/test_constraints.cpp)
  target_link_libraries(test_constraints moveit_test_utils ${MOVEIT_LIB_NAME})
endif()
