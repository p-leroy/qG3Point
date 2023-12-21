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

	target_include_directories( ${PROJECT_NAME} PRIVATE
		C:/opt/eigen-3.4.0
	)
	
	# set dependencies to necessary libraries
	# target_link_libraries( ${PROJECT_NAME} LIB1 )
endif()
