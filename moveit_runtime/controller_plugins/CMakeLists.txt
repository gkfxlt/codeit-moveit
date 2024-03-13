
#============================================================================
# controller_plugins
#============================================================================

# plugins init
set(plugins_name controller_plugins)
set(plugins_dependencies moveit_fake_controller_manager moveit_simple_controller_manager)


# add_subdirectory
add_subdirectory(moveit_fake_controller_manager)
add_subdirectory(moveit_simple_controller_manager)


# create plugins
if (DEFINED plugins_dependencies)
    add_library(${plugins_name} SHARED version.cpp)
    add_dependencies(${plugins_name} ${plugins_dependencies})
    target_link_libraries(${plugins_name} -Wl,--whole-archive ${plugins_dependencies} -Wl,--no-whole-archive)

    if (MSVC)
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
