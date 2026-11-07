option( PLUGIN_G3POINT "Install GP3Point plugin" OFF )

if ( PLUGIN_G3POINT )
	project( G3PointPlugin )

	AddPlugin(
		NAME ${PROJECT_NAME}
		TYPE standard
		SHADER_FOLDER G3Point
	)

	target_sources(G3PointPlugin
		PRIVATE
		ui/WolmanCustomPlot.ui
	)

	set(QG3POINT_PLUGIN_VERSION "0.6")

	add_subdirectory( include )
	add_subdirectory( src )
	add_subdirectory( shaders )
	add_subdirectory( ui )

	# may be needed for debug
	if(WIN32)
		target_compile_options(G3PointPlugin PRIVATE /bigobj)
	endif()

	#============
	# QCUSTOMPLOT
	target_link_libraries(${PROJECT_NAME}
		QCustomPlot
		Qt6::PrintSupport
		)

	# EIGEN
	set( EIGEN_ROOT_DIR "" CACHE PATH "Eigen root (contains the Eigen directory)" )
	if ( NOT EIGEN_ROOT_DIR )
		message( SEND_ERROR "No Eigen root directory specified (EIGEN_ROOT_DIR)" )
	else()
		message( STATUS "EIGEN_ROOT_DIR " ${EIGEN_ROOT_DIR} )
	endif()

	# OPEN3D
	option(USE_OPEN3D_WITH_G3POINT "Enable the use of Open3D with G3Point" OFF)

	if (USE_OPEN3D_WITH_G3POINT)
		find_package( Open3D REQUIRED ) # Find installed Open3D, which exports Open3D::Open3D
		target_link_libraries( ${PROJECT_NAME} Open3D::Open3D )
		message( "Open3D_DIR ${Open3D_DIR}" )
		if(WIN32)
			if(CMAKE_BUILD_TYPE STREQUAL "Debug")
				set( OPENCV_DEP_DLL_FILES
					${Open3D_DIR}/../bin/Open3D.dll
					${Open3D_DIR}/../bin/tbb12_debug.dll )
			elseif(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
				set( OPENCV_DEP_DLL_FILES
					${Open3D_DIR}/../bin/Open3D.dll
					${Open3D_DIR}/../bin/tbb12.dll )
			endif()
			copy_files( "${OPENCV_DEP_DLL_FILES}" "${CLOUDCOMPARE_DEST_FOLDER}" 1 ) #mind the quotes!
		endif()
	endif()

	if(TARGET nanoflann::nanoflann)
		target_link_libraries( ${PROJECT_NAME} nanoflann::nanoflann)
	else()
		message(error "Nanoflann not found, it should be provided by CCCoreLib" )
	endif()

	target_include_directories( ${PROJECT_NAME} PRIVATE ${EIGEN_ROOT_DIR} )

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
