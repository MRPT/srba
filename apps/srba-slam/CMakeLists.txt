# --------------------------------------------------------------
#  SRBA project
#  See docs online: https://github.com/MRPT/srba
# --------------------------------------------------------------
PROJECT(srba_slam)

FIND_PACKAGE(MRPT REQUIRED gui vision)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
FILE(GLOB LST_SRCS "*.cpp")
FILE(GLOB LST_HDRS "*.h")

ADD_EXECUTABLE(srba-slam
	${LST_SRCS}
	${LST_HDRS}
	)
	
# Precompiled hdrs for MSVC:
IF(ENABLE_PRECOMPILED_HDRS AND MSVC)
	# The "use precomp.headr" for all the files...
	set_target_properties(srba-slam
		PROPERTIES
		COMPILE_FLAGS "/Yusrba-run-generic-impl.h")

	# But for the file used to build the precomp. header:
	set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/instance_relative_graph_slam_se2.cpp"
		PROPERTIES
		COMPILE_FLAGS "/Ycsrba-run-generic-impl.h")
		
	# Skip the main.cpp from precompiled headers:
	set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/srba-slam_main.cpp" PROPERTIES COMPILE_FLAGS "/Y-")		
ENDIF(ENABLE_PRECOMPILED_HDRS AND MSVC)

SET(TMP_TARGET_NAME "srba-slam")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MRPT_LIBS})

#DeclareAppForInstall(${TMP_TARGET_NAME})

