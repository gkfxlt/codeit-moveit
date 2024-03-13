
#============================================================================
# demo_kinematics_plugin
#============================================================================

# pkg_name
set(pkg_name demo_kinematics_plugin)

# pkg_dependencies
set(pkg_dependencies moveit_kinematics_base)

# pkg_include
#get_target_property( _interface   INTERFACE_INCLUDE_DIRECTORIES)
get_target_property(moveit_plugin_loader_interface moveit_plugin_loader INTERFACE_INCLUDE_DIRECTORIES)
set(pkg_include_public include)
set(pkg_include_private ${moveit_plugin_loader_interface})
set(pkg_include_interface)

# pkg_src
set(pkg_src
        src/demo_kinematics_plugin.cpp)

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
set(MOVEIT_LIB_DEPENDS moveit_kinematics_base)

add_library(${MOVEIT_LIB_NAME} SHARED
        src/demo_kinematics_plugin.cpp
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
