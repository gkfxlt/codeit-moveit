
#============================================================================
# rostime
#============================================================================

# pkg_name
set(pkg_name rostime)


# pkg_dependencies
set(pkg_dependencies cpp_common)
set(pkg_dependencies_private stdboost)


# pkg_src
set(pkg_src
        src/duration.cpp
        src/rate.cpp
        src/time.cpp)


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
    add_executable(test_duration test/duration.cpp)
    target_link_libraries(test_duration
            ${pkg_name}
            gtest
            )
    add_test(NAME test_duration COMMAND test_duration)

    add_executable(test_time test/time.cpp)
    target_link_libraries(test_time
            ${pkg_name}
            gtest
            )
    add_test(NAME test_time COMMAND test_time)
endif ()

return()
############################################################################################


set(PKG_NAME rostime)
set(PKG_VERSION "0.6.11")
set(PKG_SOVERSION)
set(PKG_DEPENDS)

add_library(${PKG_NAME}
        src/duration.cpp
        src/rate.cpp
        src/time.cpp
        )
target_include_directories(${PKG_NAME} PUBLIC
        include
        )

# macro
PKG_DEFINE_MORE()

if (BUILD_TESTING)
    add_executable(test_duration test/duration.cpp)
    target_link_libraries(test_duration
            ${PKG_NAME}
            gtest
            )
    add_test(NAME test_duration COMMAND test_duration)

    add_executable(test_time test/time.cpp)
    target_link_libraries(test_time
            ${PKG_NAME}
            gtest
            )
    add_test(NAME test_time COMMAND test_time)
endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 2.8.3)
project(rostime)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

#find_package(catkin REQUIRED COMPONENTS cpp_common)
#find_package(Boost REQUIRED COMPONENTS date_time system thread)
#if(WIN32)
#  list(REMOVE_ITEM Boost_LIBRARIES "-lpthread")
#endif()
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS cpp_common
#  DEPENDS Boost
#)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(rostime
        src/duration.cpp
        src/rate.cpp
        src/time.cpp)

target_link_libraries(rostime ${Boost_LIBRARIES})
#if(NOT APPLE)
#  target_link_libraries(rostime ${RT_LIBRARY})
#endif()

#install(TARGETS rostime
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

#install(DIRECTORY include/
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h")

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test_duration test/duration.cpp)
    if (TARGET ${PROJECT_NAME}-test_duration)
        target_link_libraries(${PROJECT_NAME}-test_duration ${catkin_LIBRARIES} rostime)
    endif ()
    catkin_add_gtest(${PROJECT_NAME}-test_time test/time.cpp)
    if (TARGET ${PROJECT_NAME}-test_time)
        set_property(TARGET ${PROJECT_NAME}-test_time APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
        target_link_libraries(${PROJECT_NAME}-test_time ${catkin_LIBRARIES} rostime)
    endif ()
endif ()
