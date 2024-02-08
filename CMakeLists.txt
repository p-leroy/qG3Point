# CloudCompare example for standard plugins

# REPLACE ALL 'G3PointPlugin' OCCURENCES BY YOUR PLUGIN NAME
# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_G3POINT "Install example plugin" OFF )

if ( PLUGIN_G3POINT )
	project( G3PointPlugin )
	 
	AddPlugin( NAME ${PROJECT_NAME} )
		
	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( ui )

	target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)  # for mlpack

	target_include_directories( ${PROJECT_NAME} PRIVATE
		C:/opt/eigen-3.4.0
		C:/Users/PaulLeroy/miniconda3/envs/env_4_CloudCompare/include
	)
	
	# Find installed Open3D, which exports Open3D::Open3D
#	find_package(Open3D REQUIRED)
#	target_link_libraries( ${PROJECT_NAME} Open3D::Open3D)

	# On Windows if BUILD_SHARED_LIBS is enabled, copy .dll files to the executable directory
#	if(WIN32)
#		get_target_property(open3d_type Open3D::Open3D TYPE)
#		if(open3d_type STREQUAL "SHARED_LIBRARY")
#			message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
#			add_custom_command(TARGET Draw POST_BUILD
#							   COMMAND ${CMAKE_COMMAND} -E copy
#									   ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
#									   ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
#		endif()
#	endif()

endif()
