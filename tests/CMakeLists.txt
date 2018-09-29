# --------------------------------------------------------------
#  SRBA project
#  See docs online: https://github.com/MRPT/srba
# --------------------------------------------------------------
PROJECT(tests)

enable_testing()

FIND_PACKAGE(SRBA REQUIRED)
INCLUDE_DIRECTORIES(${SRBA_INCLUDE_DIRS})
FIND_PACKAGE(MRPT REQUIRED ${SRBA_REQUIRED_MRPT_MODULES})

if(MSVC)
	# Fix a "bug" in VS11 (MSVC 2012):
	add_definitions(-D_VARIADIC_MAX=10)
endif(MSVC)

SET(EXECUTABLE_OUTPUT_PATH "${${PROJECT_NAME}_BINARY_DIR}")

# Tests based on Google gtest:
# -----------------------------
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/gtest-1.8.0/include/")

file(GLOB TEST_CPP_FILES "*.cpp")

SET(lstfiles
	${TEST_CPP_FILES}
	"${PROJECT_SOURCE_DIR}/gtest-1.8.0/gtest-all.cc"
)

# Test project:
ADD_EXECUTABLE(test_srba ${lstfiles})
TARGET_LINK_LIBRARIES(test_srba ${MRPT_LIBS})

# Run it:
ADD_TEST(test_srba_build "${CMAKE_COMMAND}" --build ${CMAKE_BINARY_DIR} --target test_srba)
ADD_TEST(test_srba_run ${EXECUTABLE_OUTPUT_PATH}/test_srba)
set_tests_properties(test_srba_run PROPERTIES DEPENDS test_srba_build)
