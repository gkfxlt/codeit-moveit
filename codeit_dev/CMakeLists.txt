
#============================================================================
# codeit_dev
#============================================================================

# pkg_name
set(pkg_name codeit_dev)  # 只用于仿真


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private move_group)


# Win32 not Debug, no build codeit_dev.
if (WIN32 AND (NOT (CMAKE_BUILD_TYPE MATCHES Debug)))
    message("** Win32 can only build Debug, no build codeit_dev.")
    return()
endif ()


# pkg_src
set(pkg_src
        src/model/model_moveit.cpp
        src/function/func_moveit.cpp
        src/function/func_obsm.cpp
        src/function/func_checkjs.cpp
        src/function/func_movejs.cpp
        src/basicsystem.cpp
        src/main.cpp)


# CODEIT_ROOT
set(CODEIT_ROOT)
if (WIN32)
    set(CODEIT_ROOT ${PROJECT_SOURCE_DIR}/3rd/codeit/win32/codeit-1.0.7)
endif ()
if (UNIX)
    set(CODEIT_ROOT ${PROJECT_SOURCE_DIR}/3rd/codeit/linux-nrt/codeit-1.0.5)
    find_package(codeit REQUIRED PATHS /usr/codeit)
endif ()


#if(UNIX)
#    set(pkg_src ${pkg_src} src/sensor_rs_camera.cpp src/function/func_perception.cpp)
#    find_package(realsense2 REQUIRED)
#    find_package(OpenCV REQUIRED)
#    find_package(PCL REQUIRED)
#    include_directories(${realsense2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
#    link_libraries(${realsense2_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})
#endif(UNIX)


# add_library
link_directories(${pkg_name} PRIVATE ${CODEIT_ROOT}/lib ${CODEIT_ROOT}/lib/debug ${CODEIT_ROOT}/lib/release)
add_executable(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PRIVATE include ${CODEIT_ROOT}/include)
target_link_libraries(${pkg_name}
        -Wl,--start-group
        codeit_system codeit_model codeit_controller codeit_function codeit_sensor codeit_core codeit_vrep3rd codeit_can3rd
        -Wl,--end-group
        )

if (WIN32)
    target_link_libraries(${pkg_name} ControlCAN.lib)
endif ()
if (UNIX)
    target_link_libraries(${pkg_name} controlcan pthread atomic rt stdc++fs ${codeit_LIBRARIES})
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


# install
install(TARGETS ${pkg_name} RUNTIME DESTINATION ${INSTALL_TEST_PATH})
if (WIN32)
    install(FILES ${CODEIT_ROOT}/lib/ControlCAN.lib DESTINATION ${INSTALL_LIB_PATH})
    install(FILES ${CODEIT_ROOT}/lib/ControlCAN.dll DESTINATION ${INSTALL_LIB_PATH})
endif ()
if (UNIX)
    install(FILES ${CODEIT_ROOT}/lib/libcontrolcan.so DESTINATION ${INSTALL_LIB_PATH})
endif ()

## BUILD_TESTING
if (BUILD_TESTING)
endif ()

return()
############################################################################################
