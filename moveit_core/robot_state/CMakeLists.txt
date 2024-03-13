
#============================================================================
# moveit_robot_state
#============================================================================

# pkg_name
set(pkg_name moveit_robot_state)


# pkg_dependencies
set(pkg_dependencies moveit_robot_model moveit_transforms)
set(pkg_dependencies_private   ros_helper)


# pkg_src
set(pkg_src
		src/attached_body.cpp
		src/conversions.cpp
		src/robot_state.cpp
		src/cartesian_interpolator.cpp)


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
	MACRO(robot_state_add_test test_name test_src)
		add_executable(${test_name} ${test_src})
		add_dependencies(${test_name} ${pkg_name})
		target_link_libraries(${test_name} ${pkg_name} gtest moveit_test_utils)
		add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
	ENDMACRO()

	# test_robot_state
	robot_state_add_test(test_robot_state test/robot_state_test.cpp)

	# robot_state_benchmark
	# As an executable, this benchmark is not run as a test by default
	robot_state_add_test(robot_state_benchmark test/robot_state_benchmark.cpp)

	# test_cartesian_interpolator
	robot_state_add_test(test_cartesian_interpolator test/test_cartesian_interpolator.cpp)

	# test_robot_state_complex
	robot_state_add_test(test_robot_state_complex test/test_kinematic_complex.cpp)

	# test_aabb
	robot_state_add_test(test_aabb test/test_aabb.cpp)

endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_robot_state)
set(MOVEIT_LIB_VERSION ${${PROJECT_NAME}_VERSION})
set(MOVEIT_LIB_SOVERSION )
set(MOVEIT_LIB_DEPENDS moveit_robot_model moveit_kinematics_base moveit_transforms rostime)

add_library(${MOVEIT_LIB_NAME}
  src/attached_body.cpp
  src/conversions.cpp
  src/robot_state.cpp
  src/cartesian_interpolator.cpp
)
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
	include
)

# macro
MOVEIT_LIB_DEFINE_MORE()

if(BUILD_TESTING)
	# test_robot_state
	add_executable(test_robot_state test/robot_state_test.cpp)
	target_link_libraries(test_robot_state
		${MOVEIT_LIB_NAME}
		moveit_test_utils
		gtest
	)
	add_test(NAME test_robot_state COMMAND test_robot_state)

	# robot_state_benchmark
	# As an executable, this benchmark is not run as a test by default
	add_executable(robot_state_benchmark test/robot_state_benchmark.cpp)
	target_link_libraries(robot_state_benchmark 		
		${MOVEIT_LIB_NAME}
		moveit_test_utils
		gtest
	)
	add_test(NAME robot_state_benchmark COMMAND robot_state_benchmark)

	# test_cartesian_interpolator
	add_executable(test_cartesian_interpolator test/test_cartesian_interpolator.cpp)
	target_link_libraries(test_cartesian_interpolator
		${MOVEIT_LIB_NAME}
		moveit_test_utils
		gtest
	)
	add_test(NAME test_cartesian_interpolator COMMAND test_cartesian_interpolator)

	# test_robot_state_complex
	add_executable(test_robot_state_complex test/test_kinematic_complex.cpp)
	target_link_libraries(test_robot_state_complex
		${MOVEIT_LIB_NAME}
		moveit_test_utils
		gtest
	)
	add_test(NAME test_robot_state_complex COMMAND test_robot_state_complex)

	# test_aabb
	add_executable(test_aabb test/test_aabb.cpp)
	target_link_libraries(test_aabb
		${MOVEIT_LIB_NAME}
		moveit_test_utils
		gtest
	)
	add_test(NAME test_aabb COMMAND test_aabb)

endif()

return()
############################################################################################




set(MOVEIT_LIB_NAME moveit_robot_state)

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  # clang is picky about typeid(*solver)
  add_compile_options(-Wno-potentially-evaluated-expression)
endif()

add_library(${MOVEIT_LIB_NAME}
  src/attached_body.cpp
  src/conversions.cpp
  src/robot_state.cpp
  src/cartesian_interpolator.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_robot_model moveit_kinematics_base moveit_transforms geometric_shapes ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})

#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${MOVEIT_LIB_NAME}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# Unit tests
if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_robot_state test/robot_state_test.cpp)
  target_link_libraries(test_robot_state moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})

  # As an executable, this benchmark is not run as a test by default
  add_executable(robot_state_benchmark test/robot_state_benchmark.cpp)
  target_link_libraries(robot_state_benchmark moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME} ${GTEST_LIBRARIES})

  catkin_add_gtest(test_cartesian_interpolator test/test_cartesian_interpolator.cpp)
  target_link_libraries(test_cartesian_interpolator moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})

  catkin_add_gtest(test_robot_state_complex test/test_kinematic_complex.cpp)
  target_link_libraries(test_robot_state_complex moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})

  catkin_add_gtest(test_aabb test/test_aabb.cpp)
  target_link_libraries(test_aabb moveit_test_utils ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${MOVEIT_LIB_NAME})
endif()
