
#============================================================================
# moveit_resources
#============================================================================

if (NOT EXISTS ${PROJECT_BINARY_DIR}/bin/moveit_resources)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/moveit_resources DESTINATION ${EXECUTABLE_OUTPUT_PATH})
    message("-- Copy moveit_resources to EXECUTABLE_OUTPUT_PATH")
endif ()

############################################################################################
