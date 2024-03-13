
#============================================================================
# moveit_fake_controller_manager
#============================================================================

# pkg_name
set(pkg_name moveit_fake_controller_manager)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private   moveit_robot_state moveit_runtime yaml-cpp fake_node_handle moveit_robot_model_loader moveit_controller_manager)


# pkg_src
set(pkg_src
        src/moveit_fake_controller_manager.cpp
        src/moveit_fake_controllers.cpp)


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
endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 3.1.3)
project(moveit_fake_controller_manager)

if (NOT "${CMAKE_CXX_STANDARD}")
    set(CMAKE_CXX_STANDARD 14)
endif ()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

find_package(Boost REQUIRED thread)
find_package(catkin COMPONENTS
        moveit_core
        moveit_ros_planning
        pluginlib
        roscpp
        REQUIRED)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
        LIBRARIES
        INCLUDE_DIRS
        CATKIN_DEPENDS
        moveit_core
        moveit_ros_planning
        roscpp
)

add_library(${PROJECT_NAME}
        src/moveit_fake_controller_manager.cpp
        src/moveit_fake_controllers.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(FILES moveit_fake_controller_manager_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
