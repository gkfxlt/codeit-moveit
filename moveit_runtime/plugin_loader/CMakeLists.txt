
#============================================================================
# moveit_plugin_loader
#============================================================================

# pkg_name
set(pkg_name moveit_plugin_loader)


# pkg_dependencies
set(pkg_dependencies stdboost)
set(pkg_dependencies_private  )


if (WIN32)
    set(PLATFORM_SRCS src/shared_library_WIN32.cpp)
endif ()
if (UNIX)
    set(PLATFORM_SRCS src/shared_library_UNIX.cpp)
endif (UNIX)

# pkg_src
set(pkg_src
        src/plugin_loader.cpp
        src/plugin_loader_core.cpp
        src/meta_object.cpp
        src/multi_library_plugin_loader.cpp
        src/console.cpp
        ${PLATFORM_SRCS}
        include/plugin_loader/plugin_loader.hpp
        include/plugin_loader/plugin_loader_core.hpp
        include/plugin_loader/exceptions.hpp
        include/plugin_loader/meta_object.hpp
        include/plugin_loader/multi_library_plugin_loader.hpp
        include/plugin_loader/register_macro.hpp)


# add_library
add_library(${pkg_name} SHARED ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
set_target_properties(${pkg_name} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
target_compile_definitions(${pkg_name} PRIVATE PLUGIN_LOADER_BUILDING_DLL)
if (UNIX)
    target_link_libraries(${pkg_name} dl stdc++fs)
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
    add_library(plugin_loader_test_plugins SHARED test/plugins.cpp)
    target_link_libraries(plugin_loader_test_plugins ${pkg_name})

    add_executable(test_plugin_loader test/utest.cpp)
    target_link_libraries(test_plugin_loader ${pkg_name} gtest plugin_loader_test_plugins)
    add_test(NAME test_plugin_loader COMMAND test_plugin_loader WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 3.5)
project(plugin_loader CXX)

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

add_compile_options(-fPIC)

include_directories(include)

set(${PROJECT_NAME}_SRCS
        src/plugin_loader.cpp
        src/plugin_loader_core.cpp
        src/meta_object.cpp
        src/multi_library_plugin_loader.cpp
        src/console.cpp
        )
set(${PROJECT_NAME}_HDRS
        include/plugin_loader/plugin_loader.hpp
        include/plugin_loader/plugin_loader_core.hpp
        include/plugin_loader/exceptions.hpp
        include/plugin_loader/meta_object.hpp
        include/plugin_loader/multi_library_plugin_loader.hpp
        include/plugin_loader/register_macro.hpp
        )

if (WIN32)
    set(${PROJECT_NAME}_SRCS
            ${${PROJECT_NAME}_SRCS}
            src/shared_library_WIN32.cpp
            )
endif ()

if (UNIX)
    set(${PROJECT_NAME}_SRCS
            ${${PROJECT_NAME}_SRCS}
            src/shared_library_UNIX.cpp
            )
endif (UNIX)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_HDRS})
target_link_libraries(${PROJECT_NAME} dl)
target_compile_definitions(${PROJECT_NAME} PRIVATE "plugin_loader_BUILDING_DLL")

add_subdirectory(example)
