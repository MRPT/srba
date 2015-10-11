PROJECT(srba-tutorials)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
endif(COMMAND cmake_policy)

FIND_PACKAGE(SRBA REQUIRED)
INCLUDE_DIRECTORIES(${SRBA_INCLUDE_DIRS})
FIND_PACKAGE(MRPT REQUIRED ${SRBA_REQUIRED_MRPT_MODULES} gui graphslam)

# Set optimized building in GCC:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


MACRO(DEFINE_TUTORIAL_EXECUTABLE name)
	ADD_EXECUTABLE(${name} ${name}.cpp)
	TARGET_LINK_LIBRARIES(${name} ${MRPT_LIBS}) # Add the required libraries for linking:
	
	if(ENABLE_SOLUTION_FOLDERS)
		set_target_properties(${name} PROPERTIES FOLDER "Examples")
	endif(ENABLE_SOLUTION_FOLDERS)	
ENDMACRO(DEFINE_TUTORIAL_EXECUTABLE)

# --------------------------------------------------------------------
#  List of tutorials/examples:
# --------------------------------------------------------------------
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-monocular-se3")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-stereo-se2")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-stereo-se3")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-cartesian2d-se2")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-cartesian3d-se3")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-range-bearing-se2")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-range-bearing-se3")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-relative-graph-slam-se2")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-relative-graph-slam-se3")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-remove-observation")
DEFINE_TUTORIAL_EXECUTABLE("tutorial-srba-how-to-recover-global-map" "mrpt-graphslam")
