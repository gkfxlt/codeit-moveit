
#============================================================================
# assimp
#============================================================================

# pkg_name
set(pkg_name assimp)


# pkg_dependencies
set(pkg_dependencies)
set(pkg_dependencies_private)


# assimp_src
add_subdirectory(src)


# pkg_src
set(pkg_src)
foreach (a_src ${assimp_src})
    list(APPEND pkg_src src/${a_src})
endforeach ()


add_definitions(-DASSIMP_BUILD_NO_EXPORT)
add_definitions(-DASSIMP_BUILD_NO_C4D_IMPORTER)
add_definitions(-DOPENDDLPARSER_BUILD)
add_definitions(-DRAPIDJSON_HAS_STDSTRING=1)
add_definitions(-DRAPIDJSON_NOMEMBERITERATORCLASS)
add_definitions(-DASSIMP_IMPORTER_GLTF_USE_OPEN3DGC=1)
add_definitions(-DMINIZ_USE_UNALIGNED_LOADS_AND_STORES=0)
message(STATUS "Build an import-only version of Assimp.")


# add_library
add_library(${pkg_name} ${pkg_src})
target_include_directories(${pkg_name} PUBLIC include)
target_include_directories(${pkg_name} PRIVATE
        src
        src/contrib
        src/contrib/zlib
        src/contrib/unzip
        src/contrib/pugixml
        src/contrib/openddlparser/include
        src/contrib/rapidjson/include
        )


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
endif ()

return()
############################################################################################
