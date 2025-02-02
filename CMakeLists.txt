# CloudCompare example for standard plugins

# REPLACE ALL 'G3PointPlugin' OCCURENCES BY YOUR PLUGIN NAME
# AND ADAPT THE CODE BELOW TO YOUR OWN NEEDS!

# Add an option to CMake to control whether we build this plugin or not
option( PLUGIN_G3POINT "Install example plugin" OFF )

if ( PLUGIN_G3POINT )
	project( G3PointPlugin )
	 
	AddPlugin( NAME ${PROJECT_NAME} )

	set(QG3POINT_PLUGIN_VERSION "0.3+")

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
	
	# may be needed for debug
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")

	#============
	# QCUSTOMPLOT
	target_link_libraries(${PROJECT_NAME}
		QCustomPlot
		Qt5::PrintSupport
		)

	# Find installed Open3D, which exports Open3D::Open3D
	find_package( Open3D REQUIRED )
	target_link_libraries( ${PROJECT_NAME} Open3D::Open3D )

	message( "Open3D_DIR ${Open3D_DIR}" )
	set( OPENCV_DEP_DLL_FILES ${Open3D_DIR}/../bin/Open3D.dll)
	copy_files( "${OPENCV_DEP_DLL_FILES}" "${CLOUDCOMPARE_DEST_FOLDER}" 1) #mind the quotes!
	copy_files( "${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawGrains.vs" "${CLOUDCOMPARE_DEST_FOLDER}/shaders/G3Point" 1) #mind the quotes!
	copy_files( "${CMAKE_CURRENT_SOURCE_DIR}/shaders/DrawGrains.fs" "${CLOUDCOMPARE_DEST_FOLDER}/shaders/G3Point" 1) #mind the quotes!

	#================
	# git commit hash
	# Get the current working branch
	execute_process(
	    COMMAND git rev-parse --abbrev-ref HEAD
	    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	    OUTPUT_VARIABLE GIT_BRANCH_G3POINT
	    OUTPUT_STRIP_TRAILING_WHITESPACE)

	# Get the latest commit hash
	execute_process(
	    COMMAND git rev-parse --short HEAD
	    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	    OUTPUT_VARIABLE GIT_COMMIT_HASH_G3POINT
	    OUTPUT_STRIP_TRAILING_WHITESPACE)

	# Get the latest commit hash
	execute_process(
	    COMMAND git describe
	    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	    OUTPUT_VARIABLE GIT_TAG_G3POINT
	    OUTPUT_STRIP_TRAILING_WHITESPACE)

	# Get the current working branch
	execute_process(
	    COMMAND git rev-parse --abbrev-ref HEAD
	    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	    OUTPUT_VARIABLE GIT_BRANCH_CC
	    OUTPUT_STRIP_TRAILING_WHITESPACE)

	# Get the latest commit hash
	execute_process(
	    COMMAND git rev-parse --short HEAD
	    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	    OUTPUT_VARIABLE GIT_COMMIT_HASH_CC
	    OUTPUT_STRIP_TRAILING_WHITESPACE)

	message(${PROJECT_NAME} " " GIT_BRANCH_G3POINT " " ${GIT_BRANCH_G3POINT})
	message(${PROJECT_NAME} " " GIT_COMMIT_HASH_G3POINT " " ${GIT_COMMIT_HASH_G3POINT})
	message(${PROJECT_NAME} " " GIT_TAG_G3POINT " " ${GIT_TAG_G3POINT})
	message(${PROJECT_NAME} " " GIT_BRANCH_CC " " ${GIT_BRANCH_CC})
	message(${PROJECT_NAME} " " GIT_COMMIT_HASH_CC " " ${GIT_COMMIT_HASH_CC})
	message(${PROJECT_NAME} " " QG3POINT_VERSION " " ${QG3POINT_PLUGIN_VERSION})

	target_compile_definitions(${PROJECT_NAME} PRIVATE
	    GIT_BRANCH_CC="${GIT_BRANCH_CC}"
	    GIT_COMMMIT_HASH_CC="${GIT_COMMIT_HASH_CC}"
	    PUBLIC GIT_BRANCH_G3POINT="${GIT_BRANCH_G3POINT}"
	    GIT_COMMMIT_HASH_G3POINT="${GIT_COMMIT_HASH_G3POINT}"
	    GIT_TAG_G3POINT="${GIT_TAG_G3POINT}"
	    G3POINT_VERSION="${QG3POINT_PLUGIN_VERSION}"
	    )

endif()
