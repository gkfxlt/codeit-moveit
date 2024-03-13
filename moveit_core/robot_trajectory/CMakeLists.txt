
#============================================================================
# moveit_robot_trajectory
#============================================================================

# pkg_name
set(pkg_name moveit_robot_trajectory)


# pkg_dependencies
set(pkg_dependencies moveit_robot_state)
set(pkg_dependencies_private   tf2)


# pkg_src
set(pkg_src src/robot_trajectory.cpp)


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
	# test_robot_trajectory
	add_executable(test_robot_trajectory test/test_robot_trajectory.cpp)
	target_include_directories(test_robot_trajectory PRIVATE ${pkg_include_public})
	target_link_libraries(test_robot_trajectory ${pkg_name} gtest moveit_test_utils)
	add_test(NAME test_robot_trajectory COMMAND test_robot_trajectory WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_robot_trajectory)

add_library(${MOVEIT_LIB_NAME} src/robot_trajectory.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model moveit_robot_state ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${MOVEIT_LIB_NAME}
#  ARCHIVE  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_robot_trajectory test/test_robot_trajectory.cpp)
  target_link_libraries(test_robot_trajectory moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()
