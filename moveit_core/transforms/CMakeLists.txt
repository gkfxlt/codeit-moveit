
#============================================================================
# moveit_transforms
#============================================================================

# pkg_name
set(pkg_name moveit_transforms)


# pkg_dependencies
set(pkg_dependencies moveit_macros)
set(pkg_dependencies_private   geometric_shapes tf2)


# pkg_src
set(pkg_src src/transforms.cpp)

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
    # test_transforms
    add_executable(test_transforms test/test_transforms.cpp)
    target_link_libraries(test_transforms ${pkg_name} gtest)
    add_test(NAME test_transforms COMMAND test_transforms WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME moveit_transforms)

add_library(${MOVEIT_LIB_NAME} src/transforms.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${urdfdom_LIBRARIES} ${urdfdom_headers_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(${MOVEIT_LIB_NAME} ${catkin_EXPORTED_TARGETS})

#install(TARGETS ${MOVEIT_LIB_NAME}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
#install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

# Unit tests
if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(test_transforms test/test_transforms.cpp)
    target_link_libraries(test_transforms ${MOVEIT_LIB_NAME})
endif ()
