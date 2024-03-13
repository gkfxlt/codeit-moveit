
#============================================================================
# test_yaml-cpp
#============================================================================

# pkg_name
set(pkg_name test_yaml-cpp)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private yaml-cpp gtest)


# pkg_src
file(GLOB src_in_file *.cpp integration/*.cpp node/*.cpp)
set(pkg_src ${src_in_file})


# add_library
add_executable(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/integration
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/src)


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


return()
############################################################################################

set(PKG_NAME test_yaml-cpp)
set(PKG_VERSION "0.6.3")
set(PKG_SOVERSION )
set(PKG_DEPENDS yaml-cpp)

set(test-new-api-pattern "new-api/*.cpp")
file(GLOB test-new-api-sources ${test-new-api-pattern})

set(test-source-pattern "*.cpp" "integration/*.cpp" "node/*.cpp")
file(GLOB test-sources ${test-source-pattern})

add_library(${PKG_NAME} ${test-new-api-sources} ${test-sources})
target_include_directories(${PKG_NAME} PUBLIC
  include
)

# macro
PKG_DEFINE_MORE()

RETURN()
############################################################################################



find_package(Threads REQUIRED)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(BUILD_MOCK ON CACHE BOOL "" FORCE)
set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)

add_subdirectory(
  "${CMAKE_CURRENT_SOURCE_DIR}/gtest-1.10.0"
  "${CMAKE_CURRENT_BINARY_DIR}/prefix")

include_directories(SYSTEM "${CMAKE_CURRENT_SOURCE_DIR}/gtest-1.10.0/googletest/include")

set(test-new-api-pattern "new-api/*.cpp")
set(test-source-pattern "*.cpp" "integration/*.cpp" "node/*.cpp")
if (CMAKE_VERSION VERSION_GREATER 3.11)
  list(INSERT test-new-api-pattern 0 CONFIGURE_DEPENDS)
  list(INSERT test-source-pattern 0 CONFIGURE_DEPENDS)
endif()

file(GLOB test-new-api-sources ${test-new-api-pattern})
file(GLOB test-sources ${test-source-pattern})

add_executable(yaml-cpp-tests "")
target_sources(yaml-cpp-tests
  PRIVATE
    ${test-new-api-sources}
    ${test-sources})
target_include_directories(yaml-cpp-tests
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/integration
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/src)
target_compile_options(yaml-cpp-tests
  PRIVATE
    $<$<CXX_COMPILER_ID:Clang>:-Wno-c99-extensions -Wno-variadic-macros -Wno-sign-compare>
    $<$<CXX_COMPILER_ID:GNU>:-Wno-variadic-macros -Wno-sign-compare>)
target_link_libraries(yaml-cpp-tests
  PRIVATE
    Threads::Threads
    yaml-cpp
    gmock)

set_property(TARGET yaml-cpp-tests PROPERTY CXX_STANDARD_REQUIRED ON)
if (NOT DEFINED CMAKE_CXX_STANDARD)
  set_target_properties(yaml-cpp-tests PROPERTIES CXX_STANDARD 11)
endif()


add_test(yaml-cpp::test yaml-cpp-tests)
