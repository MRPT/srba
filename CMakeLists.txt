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
#  2. Declare targets
# =====================================
#  C++ Library: It's hdr only, so we don't need to compile it!
include_directories("${SRBA_ALL_SOURCE_DIR}/include/")
# Install!

# Apps
add_subdirectory(apps)
# Examples
add_subdirectory(examples)
# tests
add_subdirectory(tests)


# =====================================
#  3. Install, config files, etc.
# =====================================

