
#============================================================================
# roscpp_core
#============================================================================

#add_subdirectory(cpp_common)
#add_subdirectory(roscpp_traits)
#add_subdirectory(rostime)
#add_subdirectory(roscpp_serialization)

# pkg_name
set(pkg_name ros_helper)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private stdboost)


# pkg_src
set(pkg_src
        rostime/src/duration.cpp
        rostime/src/rate.cpp
        rostime/src/time.cpp
        roscpp_serialization/src/serialization.cpp)


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC cpp_common/include roscpp_traits/include rostime/include roscpp_serialization/include)


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
    # MACRO
    MACRO(ros_helper_add_test test_name test_src)
        add_executable(${test_name} ${test_src})
        add_dependencies(${test_name} ${pkg_name})
        target_link_libraries(${test_name} ${pkg_name} gtest)
        add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})
    ENDMACRO()

    # test_duration
    ros_helper_add_test(test_duration rostime/test/duration.cpp)

    # test_time
    ros_helper_add_test(test_time rostime/test/time.cpp)

    # test_serialization
    ros_helper_add_test(test_serialization roscpp_serialization/test/test_serialization.cpp)

endif ()

return()
############################################################################################
