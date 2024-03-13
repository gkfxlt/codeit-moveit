
#============================================================================
# planner_plugins
#============================================================================

# plugins init
set(plugins_name planner_plugins)
set(plugins_dependencies demo_planner_plugin moveit_ompl_interface)

add_definitions("-Wall -g")

# add_subdirectory
add_subdirectory(demo_planner_plugin)
add_subdirectory(moveit_ompl_interface)


# create plugins
if (DEFINED plugins_dependencies)
    add_library(${plugins_name} SHARED version.cpp)
    add_dependencies(${plugins_name} ${plugins_dependencies})
    target_link_libraries(${plugins_name} -Wl,--whole-archive ${plugins_dependencies} -Wl,--no-whole-archive)

    if (MSVC)
		# LIB_POSTFIX
		if (CMAKE_BUILD_TYPE MATCHES Debug)
		set(LIB_POSTFIX ${CMAKE_DEBUG_POSTFIX})
		endif ()
        FOREACH (a_lib ${plugins_dependencies})
            set_target_properties(${plugins_name} PROPERTIES LINK_FLAGS "/WHOLEARCHIVE:${a_lib}${LIB_POSTFIX}")
        ENDFOREACH ()
    endif ()


    # PLUGINS_OUTPUT_PATH
    set_target_properties(${plugins_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})
    set_target_properties(${plugins_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})
    set_target_properties(${plugins_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PLUGINS_OUTPUT_PATH})

    # install
    install(TARGETS ${plugins_name} DESTINATION ${INSTALL_PLUGINS_PATH})

endif ()

return()
############################################################################################
