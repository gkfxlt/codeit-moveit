
#============================================================================
# moveit_trajectory_execution_manager
#============================================================================

# pkg_name
set(pkg_name moveit_trajectory_execution_manager)


# pkg_dependencies
set(pkg_dependencies moveit_planning_scene_monitor moveit_controller_manager fake_node_handle)
set(pkg_dependencies_private   yaml-cpp)


# pkg_src
set(pkg_src src/trajectory_execution_manager.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
if(UNIX)
    target_link_libraries(${pkg_name} pthread)
endif()



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


set(MOVEIT_LIB_NAME moveit_trajectory_execution_manager)

add_library(${MOVEIT_LIB_NAME} src/trajectory_execution_manager.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${MOVEIT_LIB_NAME} moveit_planning_scene_monitor moveit_robot_model_loader ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${moveit_ros_planning_EXPORTED_TARGETS}) # don't build until necessary msgs are finish

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if (CATKIN_ENABLE_TESTING)

    ## This needs further cleanup before it can run
    # add_library(test_controller_manager_plugin test/test_moveit_controller_manager_plugin.cpp)
    # set_target_properties(test_controller_manager_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
    # target_link_libraries(test_controller_manager_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    #
    # find_package(rostest REQUIRED)
    # add_rostest_gtest(test_execution_manager
    #                   test/test_execution_manager.test
    #                   test/test_execution_manager.cpp)
    # target_link_libraries(test_execution_manager ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif (CATKIN_ENABLE_TESTING)
