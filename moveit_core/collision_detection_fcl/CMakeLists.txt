
#============================================================================
# moveit_collision_detection_fcl
#============================================================================

# pkg_name
set(pkg_name moveit_collision_detection_fcl)


# pkg_dependencies
set(pkg_dependencies moveit_collision_detection fcl)
set(pkg_dependencies_private  )


# pkg_src
set(pkg_src
		src/collision_common.cpp
		src/collision_env_fcl.cpp)


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
	MACRO(collision_detection_fcl_add_test test_name test_src)
		add_executable(${test_name} ${test_src})
		add_dependencies(${test_name} ${pkg_name})
		target_link_libraries(${test_name} ${pkg_name} gtest moveit_test_utils)
		add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
	ENDMACRO()

	# test_fcl_collision_detection
	collision_detection_fcl_add_test(test_fcl_collision_detection test/test_fcl_collision_detection_pr2.cpp)

	# test_fcl_collision_detection_panda
	collision_detection_fcl_add_test(test_fcl_collision_detection_panda test/test_fcl_collision_detection_panda.cpp)

endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_collision_detection_fcl)

add_library(${MOVEIT_LIB_NAME}
  src/collision_common.cpp
  src/collision_env_fcl.cpp
)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} moveit_collision_detection fcl ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${LIBFCL_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

add_library(collision_detector_fcl_plugin src/collision_detector_fcl_plugin_loader.cpp)
set_target_properties(collision_detector_fcl_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(collision_detector_fcl_plugin ${catkin_LIBRARIES} ${MOVEIT_LIB_NAME})


#install(TARGETS ${MOVEIT_LIB_NAME} collision_detector_fcl_plugin
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

#install(FILES ../collision_detector_fcl_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(moveit_resources REQUIRED)
  include_directories(${moveit_resources_INCLUDE_DIRS})

  catkin_add_gtest(test_fcl_collision_detection test/test_fcl_collision_detection_pr2.cpp)
  target_link_libraries(test_fcl_collision_detection moveit_test_utils ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

  catkin_add_gtest(test_fcl_collision_detection_panda test/test_fcl_collision_detection_panda.cpp)
  target_link_libraries(test_fcl_collision_detection_panda moveit_test_utils ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})
endif()
