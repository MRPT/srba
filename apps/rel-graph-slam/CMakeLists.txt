# --------------------------------------------------------------
#  SRBA project
#  See docs online: https://github.com/MRPT/srba
# --------------------------------------------------------------
PROJECT(rel_graph_slam)

FIND_PACKAGE(SRBA REQUIRED)
INCLUDE_DIRECTORIES(${SRBA_INCLUDE_DIRS})
FIND_PACKAGE(MRPT REQUIRED ${SRBA_REQUIRED_MRPT_MODULES} gui)

if(MSVC)
	# For MSVC to avoid the C1128 error about too large object files:
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj /D_CRT_SECURE_NO_WARNINGS")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /bigobj /D_CRT_SECURE_NO_WARNINGS")
endif(MSVC)

# Set optimized building in GCC:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")

MACRO(DEFINE_APP_EXECUTABLE name)
	ADD_EXECUTABLE(${name} ${name}.cpp)
	TARGET_LINK_LIBRARIES(${name} ${MRPT_LIBS})
	
	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(${name} PROPERTIES FOLDER "Apps")
	endif(ENABLE_SOLUTION_FOLDERS)
	
	#DeclareAppForInstall(${name})
ENDMACRO(DEFINE_APP_EXECUTABLE)

# --------------------------------------------------------------------
#  List of tutorials/examples:
# --------------------------------------------------------------------
DEFINE_APP_EXECUTABLE(rel-graph-slam-se2)

