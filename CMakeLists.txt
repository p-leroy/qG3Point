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

	# target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)  # for mlpack

	target_include_directories( ${PROJECT_NAME} PRIVATE
		C:/opt/eigen-3.4.0
		C:/Users/PaulLeroy/miniconda3/envs/env_4_CloudCompare/include
		C:/opt/open3d-devel-windows-amd64-0.18.0/include
		C:/opt/GeometricTools/GTE
	)
	
	# Find installed Open3D, which exports Open3D::Open3D
	find_package( Open3D REQUIRED )
	target_link_libraries( ${PROJECT_NAME} Open3D::Open3D )
	message( "Open3D_DIR ${Open3D_DIR}" )
	set( OPENCV_DEP_DLL_FILES ${Open3D_DIR}/../bin/Open3D.dll )
	copy_files( "${OPENCV_DEP_DLL_FILES}" "${CLOUDCOMPARE_DEST_FOLDER}" ) #mind the quotes!

endif()
