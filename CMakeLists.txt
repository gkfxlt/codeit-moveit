cmake_minimum_required(VERSION 3.12)

project(codeit-moveit)
set(${PROJECT_NAME}_VERSION "1.0.0")


# aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)


# 是否生成动态库
option(BUILD_SHARED_LIBS "build as shared library" OFF)



# 输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(PLUGINS_OUTPUT_PATH ${PROJECT_BINARY_DIR}/plugins)
# 安装路径
set(CMAKE_INSTALL_PREFIX ${PROJECT_BINARY_DIR}/install)
set(INSTALL_ROOT_PATH ${CMAKE_INSTALL_PREFIX})
set(INSTALL_BIN_PATH ${INSTALL_ROOT_PATH}/bin)
set(INSTALL_TEST_PATH ${INSTALL_ROOT_PATH}/bin)
set(INSTALL_LIB_PATH ${INSTALL_ROOT_PATH}/lib)
set(INSTALL_PLUGINS_PATH ${INSTALL_ROOT_PATH}/plugins)


# 操作系统相关
if (WIN32)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
    add_definitions(-DWIN32 -D_WINSOCK_DEPRECATED_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -D_SCL_SECURE_NO_WARNINGS -DNON_MATLAB_PARSING)
    find_package(codeit REQUIRED PATHS C:/codeit/codeit-1.3.0)
    include_directories(${codeit_INCLUDE_DIRS})
message("** Tar: ${codeit_INCLUDE_DIRS}")
include_directories(${codeit_INCLUDE_DIRS}/codeit/3rdparty)
#    SET(Boost_INCLUDE_DIR "C:\\Boost\\include\\boost-1_76")
#   SET(Boost_FILESYSTEM_LIBRARY "C:\\Boost\\lib")
endif (WIN32)
if (UNIX)
    add_definitions(-DUNIX -DNON_MATLAB_PARSING)
    find_package(codeit REQUIRED PATHS /usr/codeit)
    include_directories(${codeit_INCLUDE_DIRS})
endif (UNIX)


# cmake相关
if (CMAKE_VERSION VERSION_LESS 3.12)
    set(CONFIGURE_DEPENDS_OPTION)
else ()
    set(CONFIGURE_DEPENDS_OPTION CONFIGURE_DEPENDS)
endif ()


# LIB_POSTFIX
set(CMAKE_DEBUG_POSTFIX "d")
if (CMAKE_BUILD_TYPE MATCHES Debug)
    set(LIB_POSTFIX ${CMAKE_DEBUG_POSTFIX})
endif ()

# CTest
include(CTest)
#enable_testing()


# 开启-fPIC
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
SET(CMAKE_EXE_LINKER_FLAGS " -no-pie")


# add_definitions
add_definitions(-D_USE_MATH_DEFINES -D_HAS_TR1_NAMESPACE -D_HAS_AUTO_PTR_ETC -D_ENABLE_EXTENDED_ALIGNED_STORAGE)


# target_add_interfaces
function(target_add_interfaces TARGET OTHER_TARGETS)
    foreach (OTHER_TARGET ${OTHER_TARGETS})
        get_target_property(TMP_INCLUDES ${OTHER_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
        if (TMP_INCLUDES)
            target_include_directories(${TARGET} PUBLIC ${TMP_INCLUDES})
        endif ()
    endforeach (OTHER_TARGET)
endfunction()


# target_add_interfaces_private
function(target_add_interfaces_private TARGET OTHER_TARGETS)
    foreach (OTHER_TARGET ${OTHER_TARGETS})
        get_target_property(TMP_INCLUDES ${OTHER_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
        if (TMP_INCLUDES)
            target_include_directories(${TARGET} PRIVATE ${TMP_INCLUDES})
        endif ()
    endforeach (OTHER_TARGET)
endfunction()


# get_targets_interface
function(print_target_interface TARGET)
    get_target_property(TMP_INCLUDES ${TARGET} INTERFACE_INCLUDE_DIRECTORIES)
    message("*******************")
    message("** Target: ${TARGET}")
    message("** Interface: ${TMP_INCLUDES}")
    message("*******************")
endfunction()


# CMAKE_BUILD_TYPE
if (NOT CMAKE_BUILD_TYPE)
    message("** no set CMAKE_BUILD_TYPE. set it to Release")
    set(CMAKE_BUILD_TYPE Release)
endif ()

message("")
message("/* ********************************************************** **")
message("** CMAKE_BUILD_TYPE:  Debug Release RelWithDebInfo MinSizeRel **")
message("** Start build. CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}")
message("** ********************************************************** */")
message("")


# add_subdirectory
add_subdirectory(moveit_dependencies)
add_subdirectory(moveit_core)
add_subdirectory(moveit_runtime)
add_subdirectory(codeit_dev)

add_subdirectory(moveit_user_capabilities)
