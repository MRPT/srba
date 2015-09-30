# --------------------------------------------------------------
#  SRBA project
#  See docs online: https://github.com/MRPT/srba
# --------------------------------------------------------------
PROJECT(tests)

# Fix a "bug" in VS11 (MSVC 2012):
if(MSVC)
	add_definitions(-D_VARIADIC_MAX=10)
endif(MSVC)

SET(EXECUTABLE_OUTPUT_PATH "${${PROJECT_NAME}_BINARY_DIR}")

# Tests based on Google gtest:
# -----------------------------
INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/gtest-1.7.0-fused/fused-src/")

file(GLOB TEST_CPP_FILES "*.cpp")

SET(lstfiles 
	${TEST_CPP_FILES}
	"${PROJECT_SOURCE_DIR}/gtest-1.7.0-fused/fused-src/gtest/gtest-all.cc"
)

# Test project:
ADD_EXECUTABLE( test_srba ${lstfiles})
TARGET_LINK_LIBRARIES(test_srba ${MRPT_LIBS})

# Run it:
GET_TARGET_PROPERTY(GENERATED_EXE test_srba LOCATION)
ADD_CUSTOM_TARGET(test COMMAND "${GENERATED_EXE}" "${SRBA_ALL_SOURCE_DIR}")
ADD_DEPENDENCIES(test test_srba)

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(test_srba PROPERTIES FOLDER "unit tests")
	set_target_properties(test PROPERTIES FOLDER "unit tests")
endif(ENABLE_SOLUTION_FOLDERS)
	

