
#============================================================================
# kinematics_plugins
#============================================================================

# LIB_POSTFIX

add_definitions("-Wall -g")
# plugins init
set(plugins_name kinematics_plugins)
set(plugins_dependencies demo_kinematics_plugin kdl_kinematics_plugin)


# add_subdirectory
add_subdirectory(demo_kinematics_plugin)
add_subdirectory(kdl_kinematics_plugin)


# create plugins
if (DEFINED plugins_dependencies)
    add_library(${plugins_name} SHARED version.cpp)
    add_dependencies(${plugins_name} ${plugins_dependencies})
    target_link_libraries(${plugins_name} -Wl,--whole-archive ${plugins_dependencies} -Wl,--no-whole-archive)

    if (MSVC)
	
        FOREACH (a_lib ${plugins_dependencies})
		message("================================${a_lib}${LIB_POSTFIX}")
            set_target_properties(${plugins_name} PROPERTIES LINK_FLAGS "/WHOLEARCHIVE:${a_lib}${LIB_POSTFIX}")
        ENDFOREACH ()
    endif ()


    # PLUGINS_OUTPUT_PATH
    set_target_properties(${plugins_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})
    set_target_properties(${plugins_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})
    set_target_properties(${plugins_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})

    # install
    install(TARGETS ${plugins_name} DESTINATION ${INSTALL_PLUGINS_PATH})

endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 3.1.3)
project(moveit_kinematics)

if (NOT "${CMAKE_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD 14)
endif ()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        moveit_core
        pluginlib
        roscpp
        tf2
        tf2_kdl
        )

set(THIS_PACKAGE_INCLUDE_DIRS
        kdl_kinematics_plugin/include
        lma_kinematics_plugin/include
        srv_kinematics_plugin/include
        cached_ik_kinematics_plugin/include
        )

catkin_package(
        LIBRARIES
        INCLUDE_DIRS
        ${THIS_PACKAGE_INCLUDE_DIRS}
        CATKIN_DEPENDS
        moveit_core
        pluginlib
        roscpp
        DEPENDS
        EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        )

add_subdirectory(kdl_kinematics_plugin)
add_subdirectory(lma_kinematics_plugin)
add_subdirectory(srv_kinematics_plugin)
add_subdirectory(ikfast_kinematics_plugin)
add_subdirectory(cached_ik_kinematics_plugin)
add_subdirectory(test)

install(
        FILES
        kdl_kinematics_plugin_description.xml
        lma_kinematics_plugin_description.xml
        srv_kinematics_plugin_description.xml
        cached_ik_kinematics_plugin_description.xml
        DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION})
