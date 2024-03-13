
#============================================================================
# srdfdom
#============================================================================

# pkg_name
set(pkg_name srdfdom)


# pkg_dependencies
set(pkg_dependencies urdf)
set(pkg_dependencies_private stdboost)


# pkg_src
set(pkg_src src/model.cpp src/srdf_writer.cpp)

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

    # test_srdf_parse
    add_executable(test_srdf_parse test/test_parser.cpp)
    add_dependencies(test_srdf_parse ${pkg_name})
    target_link_libraries(test_srdf_parse ${pkg_name} gtest)
    target_compile_definitions(test_srdf_parse PRIVATE TEST_RESOURCE_LOCATION="${CMAKE_CURRENT_SOURCE_DIR}/test/resources")
    add_test(NAME test_srdf_parse COMMAND test_srdf_parse WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/resources)
endif ()

return()
############################################################################################


cmake_minimum_required(VERSION 3.0.2)
project(srdfdom)

find_package(Boost REQUIRED)

find_package(console_bridge REQUIRED)
find_package(urdfdom_headers REQUIRED)

find_package(catkin REQUIRED COMPONENTS cmake_modules urdf)

find_package(TinyXML2 REQUIRED)

include_directories(
        include
        #  ${Boost_INCLUDE_DIR}
        #  ${TinyXML2_INCLUDE_DIRS}
        #  ${catkin_INCLUDE_DIRS}
        #  ${console_bridge_INCLUDE_DIRS}
        #  ${urdfdom_headers_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)

catkin_python_setup()

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  INCLUDE_DIRS include
  DEPENDS TinyXML2 console_bridge urdfdom_headers
)

add_library(${PROJECT_NAME}
        src/model.cpp
        src/srdf_writer.cpp
        )
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME} tinyxml2 ${TinyXML2_LIBRARIES} ${catkin_LIBRARIES} ${console_bridge_LIBRARIES} ${urdfdom_headers_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(PROGRAMS
  scripts/display_srdf
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/srdf_parser.test)

    add_definitions(-DTEST_RESOURCE_LOCATION="${CMAKE_SOURCE_DIR}/test/resources")
    execute_process(COMMAND bash -c "locale -a | grep -q ^nl_NL"
            RESULT_VARIABLE TEST_LOCALE
            OUTPUT_QUIET ERROR_QUIET)
    if (TEST_LOCALE)
        message(STATUS "Locale nl_NL not available. Locale test will not be meaningful.")
    endif ()

    add_rostest_gtest(test_cpp test/srdf_parser_cpp.test test/test_parser.cpp)
    target_link_libraries(test_cpp ${catkin_LIBRARIES} ${PROJECT_NAME})
endif ()
