
#============================================================================
# moveit_trajectory_processing
#============================================================================

# pkg_name
set(pkg_name moveit_trajectory_processing)


# pkg_dependencies
set(pkg_dependencies moveit_robot_trajectory)
set(pkg_dependencies_private  )


# pkg_src
set(pkg_src
		src/iterative_time_parameterization.cpp
		src/iterative_spline_parameterization.cpp
		src/trajectory_tools.cpp
		src/time_optimal_trajectory_generation.cpp)


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
	# test_time_parameterization
	add_executable(test_time_parameterization test/test_time_parameterization.cpp)
	target_link_libraries(test_time_parameterization
			${pkg_name}
			moveit_test_utils
			gtest
			)
	add_test(NAME test_time_parameterization COMMAND test_time_parameterization WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})


	# test_time_optimal_trajectory_generation
	add_executable(test_time_optimal_trajectory_generation test/test_time_optimal_trajectory_generation.cpp)
	target_link_libraries(test_time_optimal_trajectory_generation
			${pkg_name}
			gtest
			)
	add_test(NAME test_time_optimal_trajectory_generation COMMAND test_time_optimal_trajectory_generation WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_trajectory_processing)

add_library(${MOVEIT_LIB_NAME}
  src/iterative_time_parameterization.cpp
  src/iterative_spline_parameterization.cpp
  src/trajectory_tools.cpp
  src/time_optimal_trajectory_generation.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_state moveit_robot_trajectory ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${MOVEIT_LIB_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})
  catkin_add_gtest(test_time_parameterization test/test_time_parameterization.cpp)
  target_link_libraries(test_time_parameterization moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
  catkin_add_gtest(test_time_optimal_trajectory_generation test/test_time_optimal_trajectory_generation.cpp)
  target_link_libraries(test_time_optimal_trajectory_generation ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()
