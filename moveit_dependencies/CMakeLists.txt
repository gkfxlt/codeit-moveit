
#============================================================================
# moveit_dependencies
#============================================================================

add_subdirectory(googletest)
add_subdirectory(tinyxml2)
add_subdirectory(yaml-cpp)
add_subdirectory(stdboost)

add_subdirectory(moveit_resources)

#add_subdirectory( )

add_subdirectory(roscpp_core)
add_subdirectory(ros_msgs)
add_subdirectory(tf2)

add_subdirectory(urdfdom_headers)
add_subdirectory(urdf)
add_subdirectory(srdfdom)

add_subdirectory(kdl_parser)

add_subdirectory(octomap)
add_subdirectory(libccd)
add_subdirectory(fcl)

add_subdirectory(assimp)
add_subdirectory(qhull)
add_subdirectory(geometric_shapes)


add_subdirectory(orocos_kdl)
add_subdirectory(ompl-1.5.2)

return()
