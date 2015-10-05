# --------------------------------------------------------------
#  SRBA project
#  See docs online: https://github.com/MRPT/srba
# --------------------------------------------------------------
# Root CMake script
cmake_minimum_required(VERSION 2.8.3)
project(SRBA_ALL)

# =====================================
#  1. Set-up CMake and find pkgs
# =====================================
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

# Declare lib version:
# Look for: "#define SRBA_VERSION 0xABC"
file(READ "${SRBA_ALL_SOURCE_DIR}/include/srba/version.h" STR_HPP)
STRING(REGEX MATCHALL "SRBA_VERSION.*0x[0-9,A-F]+" CMAKE_VERSION_LINE "${STR_HPP}")
STRING(REGEX MATCHALL "0x[0-9,A-F]+" SRBA_VERSION_HEX "${CMAKE_VERSION_LINE}")
STRING(REGEX REPLACE "0x(.).*" "\\1" CMAKE_SRBA_VERSION_NUMBER_MAJOR "${SRBA_VERSION_HEX}" )
STRING(REGEX REPLACE "0x.(.).*" "\\1" CMAKE_SRBA_VERSION_NUMBER_MINOR "${SRBA_VERSION_HEX}" )
STRING(REGEX REPLACE "0x..(.).*" "\\1" CMAKE_SRBA_VERSION_NUMBER_PATCH "${SRBA_VERSION_HEX}" )

SET(CMAKE_SRBA_FULL_VERSION "${CMAKE_SRBA_VERSION_NUMBER_MAJOR}.${CMAKE_SRBA_VERSION_NUMBER_MINOR}.${CMAKE_SRBA_VERSION_NUMBER_PATCH}")
MESSAGE(STATUS "Configuring for SRBA v${CMAKE_SRBA_FULL_VERSION}")

IF (CMAKE_VERSION VERSION_GREATER 2.8.0)
	SET(ENABLE_SOLUTION_FOLDERS ON CACHE BOOL "Group projects under virtual folders of the compiler IDE (e.g. VisualStudio)")
	MARK_AS_ADVANCED(ENABLE_SOLUTION_FOLDERS)
ENDIF ()
if (ENABLE_SOLUTION_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif ()

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH "${${PROJECT_NAME}_BINARY_DIR}/bin")

# MRPT:
FIND_PACKAGE(MRPT REQUIRED graphs opengl base gui)

# Shared options between GCC and CLANG:
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
    # We need pthread's
    IF (UNIX AND NOT APPLE)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    ENDIF()
    IF(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
    ENDIF()
endif()

# =====================================
#  2. Install, config files, etc.
# =====================================

# Generate CMake config file for usage without "make install"
# ---------------------------------------------------------------
SET(THE_INCLUDE_DIRECTORIES "${SRBA_ALL_SOURCE_DIR}/include")
CONFIGURE_FILE(
	"${SRBA_ALL_SOURCE_DIR}/SRBA-config.cmake.in"
    "${SRBA_ALL_BINARY_DIR}/SRBA-config.cmake" @ONLY IMMEDIATE )

SET(SRBA_DIR ${SRBA_ALL_BINARY_DIR} CACHE PATH "Directory for SRBA-config.cmake" )
MARK_AS_ADVANCED(SRBA_DIR)

# =====================================
#  3. Declare targets
# =====================================
#  C++ Library: It's hdr only, so we don't need to compile it!
include_directories("${SRBA_ALL_SOURCE_DIR}/include/")

# Anyway, we define a "dummy" target so all headers appear in the Development GUI (Visual Studio, QtCreator,...)
# A custom target (needs no real compiling)
FILE(GLOB srba_hdrs 
	"${SRBA_ALL_SOURCE_DIR}/include/*.h"
	"${SRBA_ALL_SOURCE_DIR}/include/srba/*.h"
	)
SOURCE_GROUP("Main API .h" FILES ${srba_hdrs})
FILE(GLOB srba_hdrs_models "${SRBA_ALL_SOURCE_DIR}/include/srba/models/*.h")
SOURCE_GROUP("Models .h" FILES ${srba_hdrs_models})
FILE(GLOB srba_hdrs_impl "${SRBA_ALL_SOURCE_DIR}/include/srba/impl/*.h")
SOURCE_GROUP("Impl .h" FILES ${srba_hdrs_impl})
FILE(GLOB srba_hdrs_ecps "${SRBA_ALL_SOURCE_DIR}/include/srba/ecps/*.h")
SOURCE_GROUP("Edge creation policies .h" FILES ${srba_hdrs_ecps})

set(srba_all_srcs ${srba_hdrs} ${srba_hdrs_models} ${srba_hdrs_impl} ${srba_hdrs_ecps})
add_custom_target(srba DEPENDS ${srba_all_srcs} SOURCES ${srba_all_srcs})
set_target_properties(srba PROPERTIES FOLDER "SRBA lib")

# Install!

# Apps
add_subdirectory(apps)
# Examples
add_subdirectory(examples)
# tests
add_subdirectory(tests)


