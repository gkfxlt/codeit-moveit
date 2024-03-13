
#============================================================================
# kdl_kinematics_plugin
#============================================================================

# pkg_name
set(pkg_name kdl_kinematics_plugin)

# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private   moveit_plugin_loader moveit_robot_state kdl_parser)


# pkg_src
set(pkg_src
        src/kdl_kinematics_plugin.cpp
        src/chainiksolver_vel_mimic_svd.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
#target_include_directories(${pkg_name} PUBLIC include ${PROJECT_SOURCE_DIR}/3rd/orocos_kdl/include)
#target_link_libraries(${pkg_name} ${PROJECT_SOURCE_DIR}/3rd/orocos_kdl/lib/unix/liborocos-kdl.a)


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


set(MOVEIT_LIB_NAME moveit_kdl_kinematics_plugin)

add_library(${MOVEIT_LIB_NAME}
        src/kdl_kinematics_plugin.cpp
        src/chainiksolver_vel_mimic_svd.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
