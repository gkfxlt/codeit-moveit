
#============================================================================
# urdf_parser_plugin
#============================================================================

# pkg_name
set(pkg_name urdf_parser_plugin)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private)


# pkg_src
set(pkg_src)


# add_library
add_library(${pkg_name} INTERFACE)
target_include_directories(${pkg_name} INTERFACE include)


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


cmake_minimum_required(VERSION 3.5)
project(urdf_parser_plugin)

find_package(ament_cmake_ros REQUIRED)
find_package(urdfdom_headers REQUIRED)

# Create and export a header-only target
add_library(urdf_parser_plugin INTERFACE)
target_include_directories(urdf_parser_plugin INTERFACE
        $<INSTALL_INTERFACE:include>
        )
ament_target_dependencies(urdf_parser_plugin INTERFACE
        "urdfdom_headers"
        )
install(TARGETS urdf_parser_plugin EXPORT urdf_parser_plugin-export)
ament_export_targets(urdf_parser_plugin-export)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_package()
