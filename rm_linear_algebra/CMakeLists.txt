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
message("   Including robotMath")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(include)

add_library(${PROJECT_NAME}
  src/LinearAlgebra.cpp
)


if (TEST)
################################
# GTest
################################
ADD_SUBDIRECTORY (../gtest-1.7.0 gtest)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

################################
# Unit Tests
################################
# Add test cpp file
add_executable( runUnitTests test/test_main.cpp test/SkewMatrixFromVectorTest.cpp )
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTests ${PROJECT_NAME} gtest gtest_main)
add_test( runUnitTests runUnitTests )

endif (TEST)