
#============================================================================
# fake_node_handle
#============================================================================

# pkg_name
set(pkg_name fake_node_handle)


# pkg_dependencies
set(pkg_dependencies ros_helper)
set(pkg_dependencies_private   stdboost)


# pkg_src
set(pkg_src
        src/fake_node_handle.cpp
        src/param.cpp
        src/topic.cpp
        src/service.cpp
        src/action.cpp
        src/timer.cpp)


# add_library
add_library(${pkg_name} SHARED ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
set_target_properties(${pkg_name} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
target_compile_definitions(${pkg_name} PRIVATE FAKE_NODE_HANDLE_BUILDING_DLL)
if (UNIX)
    target_link_libraries(${pkg_name} pthread)
endif ()


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


# OUTPUT_DIRECTORY
set_target_properties(${pkg_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
set_target_properties(${pkg_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
set_target_properties(${pkg_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

# install
install(TARGETS ${pkg_name} DESTINATION ${INSTALL_LIB_PATH})


## BUILD_TESTING
if (BUILD_TESTING)
    # libA
    add_library(test_lib_a SHARED test/libA.cpp)
    add_dependencies(test_lib_a ${pkg_name} moveit_plugin_loader)
    target_link_libraries(test_lib_a ${pkg_name} moveit_plugin_loader)

    # libB
    add_library(test_lib_b SHARED test/libB.cpp)
    add_dependencies(test_lib_b ${pkg_name} moveit_plugin_loader)
    target_link_libraries(test_lib_b ${pkg_name} moveit_plugin_loader)

    # test_fake_node_handle
    add_executable(test_fake_node_handle test/test_fake_node_handle.cpp)
    target_link_libraries(test_fake_node_handle ${pkg_name} gtest moveit_plugin_loader ros_msgs)
    add_test(NAME test_fake_node_handle COMMAND test_fake_node_handle WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

endif ()

return()
############################################################################################
