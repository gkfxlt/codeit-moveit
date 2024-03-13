
#============================================================================
# moveit_constraint_samplers
#============================================================================

# pkg_name
set(pkg_name moveit_constraint_samplers)


# pkg_dependencies
set(pkg_dependencies moveit_kinematic_constraints moveit_planning_scene)
set(pkg_dependencies_private  )


# pkg_src
set(pkg_src
        src/constraint_sampler.cpp
        src/constraint_sampler_manager.cpp
        src/constraint_sampler_tools.cpp
        src/default_constraint_samplers.cpp
        src/union_constraint_sampler.cpp)


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
    add_executable(test_constraint_samplers test/test_constraint_samplers.cpp test/pr2_arm_kinematics_plugin.cpp test/pr2_arm_ik.cpp)
    add_dependencies(test_constraint_samplers ${pkg_name})
    target_link_libraries(test_constraint_samplers ${pkg_name} gtest kdl_parser moveit_test_utils  )
    add_test(NAME test_constraint_samplers COMMAND test_constraint_samplers WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_constraint_samplers)

add_library(${MOVEIT_LIB_NAME}
        src/constraint_sampler.cpp
        src/constraint_sampler_manager.cpp
        src/constraint_sampler_tools.cpp
        src/default_constraint_samplers.cpp
        src/union_constraint_sampler.cpp
        )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME}
        moveit_robot_state
        moveit_kinematic_constraints
        moveit_kinematics_base
        moveit_planning_scene
        ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${MOVEIT_LIB_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
    find_package(orocos_kdl REQUIRED)
    find_package(angles REQUIRED)
    find_package(tf2_kdl REQUIRED)

    include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS} ${angles_INCLUDE_DIRS} ${tf2_kdl_INCLUDE_DIRS})

    catkin_add_gtest(test_constraint_samplers
            test/test_constraint_samplers.cpp
            test/pr2_arm_kinematics_plugin.cpp
            test/pr2_arm_ik.cpp
            )

    target_link_libraries(test_constraint_samplers
            moveit_test_utils
            ${MOVEIT_LIB_NAME}
            ${catkin_LIBRARIES}
            ${angles_LIBRARIES}
            ${orocos_kdl_LIBRARIES}
            ${urdfdom_LIBRARIES}
            ${urdfdom_headers_LIBRARIES}
            )
endif ()
