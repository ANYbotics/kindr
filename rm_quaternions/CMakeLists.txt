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

add_definitions(-std=c++0x)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)


find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(include)
include_directories(../rm_common/include)
include_directories(../rm_rotations/include)

if (BUILD_GTEST)
################################
# Unit Tests
################################
################################
# GTest
################################

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(test)

include_directories(
#	../../schweizer-messer/sm_common/include
#	../../schweizer-messer/sm_eigen/include
#	../../schweizer-messer/sm_random/include
#	../../schweizer-messer/sm_timing/include
)
set(TEST_SRCS
	test/test_main.cpp
	test/QuaternionsTest.cpp
)

# Add test cpp file
add_executable( runUnitTestsQuaternions  ${TEST_SRCS})
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTestsQuaternions  gtest_main gtest pthread  ${PROJECT_NAME} )
add_test( runUnitTestsQuaternions runUnitTestsQuaternions )
endif()
