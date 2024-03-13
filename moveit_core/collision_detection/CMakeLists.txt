
#============================================================================
# moveit_collision_detection
#============================================================================

# pkg_name
set(pkg_name moveit_collision_detection)


# pkg_dependencies
set(pkg_dependencies moveit_robot_state)
set(pkg_dependencies_private   tf2)


# pkg_src
set(pkg_src
		src/allvalid/collision_env_allvalid.cpp
		src/collision_matrix.cpp
		src/collision_octomap_filter.cpp
		src/collision_tools.cpp
		src/world.cpp
		src/world_diff.cpp
		src/collision_env.cpp)


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
	MACRO(collision_detection_add_test test_name test_src)
		add_executable(${test_name} ${test_src})
		add_dependencies(${test_name} ${pkg_name})
		target_link_libraries(${test_name} ${pkg_name} gtest)
		add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
	ENDMACRO()

	# test_world
	collision_detection_add_test(test_world test/test_world.cpp)

	# test_world_diff
	collision_detection_add_test(test_world_diff test/test_world_diff.cpp)

	# test_all_valid
	collision_detection_add_test(test_all_valid test/test_all_valid.cpp)
endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_collision_detection)

add_library(${MOVEIT_LIB_NAME}
  src/allvalid/collision_env_allvalid.cpp
  src/collision_matrix.cpp
  src/collision_octomap_filter.cpp
  src/collision_tools.cpp
  src/world.cpp
  src/world_diff.cpp
  src/collision_env.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

# unit tests
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_world test/test_world.cpp)
  target_link_libraries(test_world ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

  catkin_add_gtest(test_world_diff test/test_world_diff.cpp)
  target_link_libraries(test_world_diff ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

  catkin_add_gtest(test_all_valid test/test_all_valid.cpp)
  target_link_libraries(test_all_valid ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
endif()


#install(TARGETS ${MOVEIT_LIB_NAME}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
