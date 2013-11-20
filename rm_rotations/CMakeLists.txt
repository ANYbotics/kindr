#-------------------------------------------------------------------------------
#
#  Robot Math Library
#
# Autonomous Systems Lab
# ETH Zurich
#
# Author(s): Christian Gehring
# email    : gehrinch@ethz.ch
# Date     : 18-Jan-2013
#
#-------------------------------------------------------------------------------
# Project configuration
cmake_minimum_required (VERSION 2.8)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Release)

add_definitions(-std=c++0x)

find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(include)
include_directories(../rm_common/include)
include_directories(../rm_quaternions/include)

add_executable( rot_example  ./src/example.cpp)

#if (TEST)
################################
# Unit Tests
################################
################################
# GTest
################################

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(test)

include_directories(
	#../../schweizer-messer/sm_common/include
	#../../schweizer-messer/sm_eigen/include
	#../../schweizer-messer/sm_random/include
	#../../schweizer-messer/sm_timing/include
)
set(TEST_SRCS
	test/test_main.cpp
	test/RotationsTest.cpp
	#test/QuaternionsTest.cpp
	#test/SimulationSubfunctions.cpp
	#../../schweizer-messer/sm_random/src/random.cpp
	#../../schweizer-messer/sm_eigen/src/random.cpp
	#../../schweizer-messer/sm_timing/src/Timer.cpp
)

# Add test cpp file
add_executable( runUnitTestsRotations  ${TEST_SRCS})
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTestsRotations  gtest_main gtest pthread  ${PROJECT_NAME} )
add_test( runUnitTestsRotations runUnitTestsRotations )
#endif()
