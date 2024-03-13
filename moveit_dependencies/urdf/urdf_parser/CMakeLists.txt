
#============================================================================
# urdf_parser
#============================================================================

# pkg_name
set(pkg_name urdf_parser)


# pkg_dependencies
set(pkg_dependencies urdfdom_headers)
set(pkg_dependencies_private   tinyxml2)


# pkg_src
set(pkg_src src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include ${PROJECT_SOURCE_DIR}/moveit_dependencies/log_helper/include/ )


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
    add_executable(test_urdf_parser test/urdf_unit_test.cpp)
    add_dependencies(test_urdf_parser ${pkg_name})
    target_link_libraries(test_urdf_parser ${pkg_name} gtest_main)
    add_test(NAME test_urdf_parser COMMAND test_urdf_parser WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
endif ()

return()
############################################################################################


include_directories(include)

add_library(urdfdom_world src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp src/world.cpp)
target_link_libraries(urdfdom_world ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES} console_bridge tinyxml2)
#set_target_properties(urdfdom_world PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_model src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp)
target_link_libraries(urdfdom_model ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES} console_bridge tinyxml2)
#set_target_properties(urdfdom_model PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_sensor src/urdf_sensor.cpp)
target_link_libraries(urdfdom_sensor urdfdom_model ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES} console_bridge tinyxml2)
#set_target_properties(urdfdom_sensor PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

add_library(urdfdom_model_state src/urdf_model_state.cpp src/twist.cpp)
target_link_libraries(urdfdom_model_state ${TinyXML_LIBRARIES} ${console_bridge_LIBRARIES} console_bridge tinyxml2)
#set_target_properties(urdfdom_model_state PROPERTIES SOVERSION ${URDF_MAJOR_MINOR_VERSION})

# --------------------------------

#add_executable(check_urdf src/check_urdf.cpp)
#target_link_libraries(check_urdf urdfdom_model urdfdom_world)

#add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
#target_link_libraries(urdf_to_graphiz urdfdom_model)

# urdf_mem_test is a binary for testing, not a unit test
#add_executable(urdf_mem_test test/memtest.cpp)
#target_link_libraries(urdf_mem_test urdfdom_model)

#add_subdirectory(test)

#INSTALL(TARGETS urdfdom_model DESTINATION ${CMAKE_INSTALL_LIBDIR})
#INSTALL(TARGETS urdfdom_world DESTINATION ${CMAKE_INSTALL_LIBDIR})
#INSTALL(TARGETS check_urdf urdf_to_graphiz urdf_mem_test
#  DESTINATION ${CMAKE_INSTALL_BINDIR})
#INSTALL(TARGETS urdfdom_sensor DESTINATION ${CMAKE_INSTALL_LIBDIR})
#INSTALL(TARGETS urdfdom_model_state DESTINATION ${CMAKE_INSTALL_LIBDIR})
#INSTALL(DIRECTORY include/urdf_parser DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
