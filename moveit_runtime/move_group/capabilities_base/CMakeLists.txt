
#============================================================================
# move_group_capabilities_base
#============================================================================

# pkg_name
set(pkg_name move_group_capabilities_base)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private   moveit_planning_pipeline moveit_macros moveit_planning_interface moveit_plan_execution)


# pkg_src
set(pkg_src
        src/move_group_context.cpp
        src/move_group_capability.cpp)


# add_library
add_library(${pkg_name} SHARED ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
set_target_properties(${pkg_name} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
target_compile_definitions(${pkg_name} PRIVATE CAPABILITY_BASE_BUILDING_DLL)


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
endif ()

return()
############################################################################################
