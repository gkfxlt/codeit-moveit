
#============================================================================
# ros_adapter_capability
#============================================================================

execute_process(
        COMMAND lsb_release -a
        COMMAND grep Release
        COMMAND awk "{print $2}"
        TIMEOUT 1
        OUTPUT_VARIABLE UBUNTU_VERSION
        OUTPUT_STRIP_TRAILING_WHITESPACE
)
message("@@ UBUNTU_VERSION = ${UBUNTU_VERSION}")
string(REPLACE "." "_" UBUNTU_VERSION "${UBUNTU_VERSION}")


# pkg_name
set(pkg_name ros_adapter_capability_${UBUNTU_VERSION})


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private move_group_capabilities_base fake_node_handle moveit_plugin_loader)


# 是否生成ROS_ADAPTER
find_package(catkin QUIET COMPONENTS actionlib roscpp)
if (catkin_FOUND)
    message("** catkin found. build ros_adapter_capability.")
else()
    message("** catkin not found. no build ros_adapter_capability.")
    return()
endif ()


# pkg_src
set(pkg_src
        src/ros_adapter_capability.cpp
        src/param_adapter.cpp
        src/topic_adapter.cpp
        src/service_adapter.cpp
        src/action_adapter.cpp)


# add_library
add_library(${pkg_name} SHARED ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include include/normalization_msgs)
set_target_properties(${pkg_name} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
target_compile_definitions(${pkg_name} PRIVATE ROS_ADAPTER_BUILDING_DLL)
target_compile_definitions(${pkg_name} PRIVATE UBUNTU_${UBUNTU_VERSION})
target_include_directories(${pkg_name} PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${pkg_name} ${catkin_LIBRARIES})


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


# PLUGINS_OUTPUT_PATH
set_target_properties(${pkg_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})
set_target_properties(${pkg_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})

# install
install(TARGETS ${pkg_name}
        LIBRARY DESTINATION ${INSTALL_PLUGINS_PATH}
        ARCHIVE DESTINATION ${INSTALL_PLUGINS_PATH})


## BUILD_TESTING
if (BUILD_TESTING)
endif ()

return()
############################################################################################


set(MOVEIT_LIB_NAME ros_adapter_capability)
set(MOVEIT_LIB_VERSION ${${PROJECT_NAME}_VERSION})
set(MOVEIT_LIB_SOVERSION)
set(MOVEIT_LIB_DEPENDS moveit_runtime)

add_library(${MOVEIT_LIB_NAME} SHARED
        src/ros_adapter_capability.cpp
        src/param_adapter.cpp
        src/topic_adapter.cpp
        src/service_adapter.cpp
        src/action_adapter.cpp
        )
target_include_directories(${MOVEIT_LIB_NAME} PUBLIC
        include
        /opt/ros/noetic/include
        )
target_link_directories(ros_adapter_capability PUBLIC
        /opt/ros/noetic/lib
        )
#target_link_libraries(ros_adapter_capability moveit_move_group_capabilities_base fake_node_handle roscpp roscpp_serialization actionlib rosconsole)
target_link_libraries(ros_adapter_capability libactionlib.so libroscpp.so moveit_move_group_capabilities_base fake_node_handle moveit_plugin_loader)

MOVEIT_LIB_DEFINE_MORE()

add_library(test_action_helper test/test_action_helper.cpp)
target_link_libraries(test_action_helper fake_node_handle)
target_include_directories(test_action_helper PRIVATE
        /opt/ros/noetic/include
        #        ${PROJECT_SOURCE_DIR}/3rd/include/noetic
        #        ${PROJECT_SOURCE_DIR}/moveit_dependencies/roscpp_core/rostime/include
        #        ${PROJECT_SOURCE_DIR}/moveit_dependencies/roscpp_core/roscpp_traits/include
        #        ${PROJECT_SOURCE_DIR}/moveit_dependencies/roscpp_core/roscpp_serialization/include
        )

add_executable(test_ros_adapter test/main.cpp)
target_link_libraries(test_ros_adapter ros_adapter_capability pthread test_action_helper)

return()
############################################################################################
