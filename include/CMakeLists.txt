
target_sources( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_LIST_DIR}/ActionA.h
		${CMAKE_CURRENT_LIST_DIR}/G3Point.h
		${CMAKE_CURRENT_LIST_DIR}/G3PointDialog.h
		${CMAKE_CURRENT_LIST_DIR}/GrainsAsEllipsoids.h
		${CMAKE_CURRENT_LIST_DIR}/qG3PointDisclaimer.h
		${CMAKE_CURRENT_LIST_DIR}/WolmanCustomPlot.h
)

target_include_directories( ${PROJECT_NAME}
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}
)
