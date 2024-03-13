
#============================================================================
# demo_planner_plugin
#============================================================================

# pkg_name
set(pkg_name demo_planner_plugin)

# pkg_dependencies
set(pkg_dependencies moveit_planning_interface)

# pkg_include
#get_target_property( _interface   INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(moveit_plugin_loader_interface moveit_plugin_loader INTERFACE_INCLUDE_DIRECTORIES)
set(pkg_include_public include)
set(pkg_include_private ${moveit_plugin_loader_interface})
set(pkg_include_interface)

# pkg_src
set(pkg_src
        src/demo_planner_plugin.cpp)

# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC ${pkg_include_public})
target_include_directories(${pkg_name} PRIVATE ${pkg_include_private})
target_include_directories(${pkg_name} INTERFACE ${pkg_include_interface})

# add_dependencies
if (DEFINED pkg_dependencies)
    add_dependencies(${pkg_name} ${pkg_dependencies})
    target_link_libraries(${pkg_name} ${pkg_dependencies})
endif ()

## BUILD_TESTING
if (BUILD_TESTING)
endif ()

return()
############################################################################################

STRING(REGEX REPLACE ".*/(.*)" "\\1" CURRENT_FOLDER ${CMAKE_CURRENT_SOURCE_DIR})

set(MOVEIT_LIB_NAME ${CURRENT_FOLDER})
set(MOVEIT_LIB_VERSION ${${PROJECT_NAME}_VERSION})
set(MOVEIT_LIB_SOVERSION)
set(MOVEIT_LIB_DEPENDS moveit_planning_interface)

add_library(${MOVEIT_LIB_NAME}
        src/demo_planner_plugin.cpp
        )
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
        include
        )

# macro
MOVEIT_LIB_DEFINE_MORE()

if (BUILD_TESTING)
endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_ompl_interface)

add_library(${MOVEIT_LIB_NAME}
        src/ompl_interface.cpp
        src/planning_context_manager.cpp
        src/model_based_planning_context.cpp
        src/parameterization/model_based_state_space.cpp
        src/parameterization/model_based_state_space_factory.cpp
        src/parameterization/joint_space/joint_model_state_space.cpp
        src/parameterization/joint_space/joint_model_state_space_factory.cpp
        src/parameterization/work_space/pose_model_state_space.cpp
        src/parameterization/work_space/pose_model_state_space_factory.cpp
        src/detail/threadsafe_state_storage.cpp
        src/detail/state_validity_checker.cpp
        src/detail/projection_evaluators.cpp
        src/detail/goal_union.cpp
        src/detail/constraints_library.cpp
        src/detail/constrained_sampler.cpp
        src/detail/constrained_valid_state_sampler.cpp
        src/detail/constrained_goal_sampler.cpp
        )
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

find_package(OpenMP REQUIRED)

target_link_libraries(${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

add_executable(moveit_generate_state_database scripts/generate_state_database.cpp)
target_link_libraries(moveit_generate_state_database ${MOVEIT_LIB_NAME})
set_target_properties(moveit_generate_state_database PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
set_target_properties(moveit_generate_state_database PROPERTIES OUTPUT_NAME "generate_state_database")

add_library(moveit_ompl_planner_plugin src/ompl_planner_manager.cpp)
set_target_properties(moveit_ompl_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_ompl_planner_plugin ${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} moveit_ompl_planner_plugin
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS moveit_generate_state_database
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_state_space test/test_state_space.cpp)
    target_link_libraries(test_state_space ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES})
    set_target_properties(test_state_space PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")

    find_package(rostest REQUIRED)
    find_package(tf2_eigen REQUIRED)

    add_rostest_gtest(test_planning_context_manager
            test/test_planning_context_manager.test
            test/test_planning_context_manager.cpp)
    target_link_libraries(test_planning_context_manager ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${tf2_eigen_LIBRARIES})

    catkin_add_gtest(test_state_validity_checker test/test_state_validity_checker.cpp)
    target_link_libraries(test_state_validity_checker ${MOVEIT_LIB_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES})
    set_target_properties(test_state_validity_checker PROPERTIES LINK_FLAGS "${OpenMP_CXX_FLAGS}")
endif ()
