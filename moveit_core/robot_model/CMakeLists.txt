
#============================================================================
# moveit_robot_model
#============================================================================

# pkg_name
set(pkg_name moveit_robot_model)


# pkg_dependencies
set(pkg_dependencies moveit_profiler moveit_exceptions moveit_kinematics_base srdfdom)
set(pkg_dependencies_private   geometric_shapes)


# pkg_src
set(pkg_src
		src/aabb.cpp
		src/fixed_joint_model.cpp
		src/floating_joint_model.cpp
		src/joint_model.cpp
		src/joint_model_group.cpp
		src/link_model.cpp
		src/planar_joint_model.cpp
		src/prismatic_joint_model.cpp
		src/revolute_joint_model.cpp
		src/robot_model.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include ${PROJECT_SOURCE_DIR}/3rd/eigen3)


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
	# test_robot_model
	add_executable(test_robot_model test/test.cpp)
	add_dependencies(test_robot_model ${MOVEIT_LIB_NAME} moveit_test_utils)
	target_link_libraries(test_robot_model
			${pkg_name}
			moveit_test_utils
			gtest
			)
	add_test(NAME test_robot_model COMMAND test_robot_model WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

return()
############################################################################################



set(MOVEIT_LIB_NAME moveit_robot_model)

add_library(${MOVEIT_LIB_NAME}
  src/aabb.cpp
  src/fixed_joint_model.cpp
  src/floating_joint_model.cpp
  src/joint_model.cpp
  src/joint_model_group.cpp
  src/link_model.cpp
  src/planar_joint_model.cpp
  src/prismatic_joint_model.cpp
  src/revolute_joint_model.cpp
  src/robot_model.cpp
  )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_profiler moveit_exceptions moveit_kinematics_base ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_robot_model test/test.cpp)
  target_link_libraries(test_robot_model moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()

#install(TARGETS ${MOVEIT_LIB_NAME}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
