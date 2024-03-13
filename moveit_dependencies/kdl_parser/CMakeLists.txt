
#============================================================================
# kdl_parser
#============================================================================

# pkg_name
set(pkg_name kdl_parser)


# pkg_dependencies
set(pkg_dependencies urdfdom_headers orocos_kdl)
set(pkg_dependencies_private   urdf_parser)


# pkg_src
set(pkg_src src/kdl_parser.cpp)

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
    # MACRO
    MACRO(kdl_parser_add_test test_name test_src)
        add_executable(${test_name} ${test_src})
        add_dependencies(${test_name} ${pkg_name})
        target_compile_definitions(${test_name} PRIVATE TEST_RESOURCE_LOCATION="${CMAKE_CURRENT_SOURCE_DIR}/test/resources")
        target_include_directories(${pkg_name} PUBLIC ${PROJECT_SOURCE_DIR}/3rd/eigen3)
        target_link_libraries(${test_name} ${pkg_name} gtest)
        add_test(NAME ${test_name} COMMAND ${test_name} WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    ENDMACRO()

    # test_kdl_parser
    kdl_parser_add_test(test_kdl_parser test/test_kdl_parser.cpp)

    # test_kdl_parser
    kdl_parser_add_test(test_inertia_rpy test/test_inertia_rpy.cpp)
endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 3.7.2)

project(kdl_parser)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin QUIET
        COMPONENTS rosconsole cmake_modules
        )

if (NOT catkin_FOUND)
    # use local copies of FindTinyXML.cmake and FindTinyXML2.cmake from
    # 'cmake_modules' (https://github.com/ros/cmake_modules)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
endif ()

find_package(urdfdom REQUIRED)
include_directories(${urdfdom_INCLUDE_DIRS})

find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)
find_package(TinyXML2 REQUIRED)

# check for rosconsole
# We check additionally for catkin to distinguish between an "official" ROS distribution
# and the one provided in the distribution's repository.
find_package(rosconsole QUIET)
if (rosconsole_FOUND AND catkin_FOUND)
    add_definitions(-DHAS_ROS)
endif ()

find_package(urdf QUIET)
if (urdf_FOUND)
    add_definitions(-DHAS_URDF)
    include_directories(${urdf_INCLUDE_DIRS})
endif ()

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

if (catkin_FOUND)
    link_directories(${catkin_LIBRARY_DIRS})
    include_directories(${catkin_INCLUDE_DIRS})

    catkin_package(
            LIBRARIES ${PROJECT_NAME} ${orocos_kdl_LIBRARIES}
            INCLUDE_DIRS include
            CATKIN_DEPENDS rosconsole urdf
            DEPENDS orocos_kdl TinyXML TinyXML2
    )
endif ()

add_library(${PROJECT_NAME} SHARED src/kdl_parser.cpp)
target_link_libraries(${PROJECT_NAME}
        ${TinyXML_LIBRARIES} ${TinyXML2_LIBRARIES} ${orocos_kdl_LIBRARIES}
        )

target_link_libraries(${PROJECT_NAME} ${urdfdom_LIBRARIES})

if (catkin_FOUND)
    target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
endif ()

if (urdf_FOUND)
    target_link_libraries(${PROJECT_NAME} ${urdf_LIBRARIES})
endif ()

if (WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE "KDL_PARSER_BUILDING_DLL")
endif ()

add_executable(check_kdl_parser src/check_kdl_parser.cpp)
target_link_libraries(check_kdl_parser ${PROJECT_NAME})

if (catkin_FOUND AND CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS roscpp rostest)
    add_rostest_gtest(test_kdl_parser test/test_kdl_parser.launch test/test_kdl_parser.cpp)
    target_link_libraries(test_kdl_parser ${PROJECT_NAME})

    add_rostest_gtest(test_inertia_rpy test/test_inertia_rpy.launch test/test_inertia_rpy.cpp)
    target_link_libraries(test_inertia_rpy ${PROJECT_NAME})
endif ()

if (catkin_FOUND)
    # How does CATKIN do this?
    #rosbuild_add_rostest(${PROJECT_SOURCE_DIR}/test/test_kdl_parser.launch)

    # Install library
    install(TARGETS ${PROJECT_NAME}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

    install(DIRECTORY include/${PROJECT_NAME}/
            DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
else ()
    install(TARGETS ${PROJECT_NAME}
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
    install(DIRECTORY include/${PROJECT_NAME}/
            DESTINATION include)
endif ()
