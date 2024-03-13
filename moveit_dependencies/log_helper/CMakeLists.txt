
#============================================================================
#  
#============================================================================

# pkg_name
set(pkg_name  )

# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private)


# pkg_src
set(pkg_src src/ .cpp)


# CODEIT_ROOT
set(CODEIT_ROOT)
if (WIN32)
    set(CODEIT_ROOT ${PROJECT_SOURCE_DIR}/3rd/codeit/win32/codeit-1.0.7)
endif ()
if (UNIX)
    set(CODEIT_ROOT ${PROJECT_SOURCE_DIR}/3rd/codeit/linux-nrt/codeit-1.0.5)
endif ()


# USE_STD_OUT
set(USE_STD_OUT 0)

if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(USE_STD_OUT 1)
elseif (WIN32)
    set(USE_STD_OUT 1)
endif ()


# add_library
add_library(${pkg_name} SHARED ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
target_include_directories(${pkg_name} PRIVATE ${CODEIT_ROOT}/include)
target_compile_definitions(${pkg_name} PRIVATE LOG_HELPER_BUILDING_DLL)
target_compile_definitions(${pkg_name} PRIVATE ${CMAKE_BUILD_TYPE})
target_compile_definitions(${pkg_name} PRIVATE USE_STD_OUT=${USE_STD_OUT})
set_target_properties(${pkg_name} PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
target_link_directories(${pkg_name} PRIVATE ${CODEIT_ROOT}/lib/debug)
target_link_directories(${pkg_name} PRIVATE ${CODEIT_ROOT}/lib/release)

if (NOT USE_STD_OUT)
    target_link_libraries(${pkg_name} PRIVATE codeit_core)
    if (UNIX)
         target_link_libraries(${pkg_name} PRIVATE stdc++fs)
    endif ()
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
    add_executable(test_log test/test.cpp)
    add_dependencies(test_log ${pkg_name})
    target_link_libraries(test_log
            ${pkg_name}
            gtest
            )
    add_test(NAME test_log COMMAND test_log)
endif ()

return()
############################################################################################
