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

project(robotMath)


# Set Build Type
#   * None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
#   * Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
#   * Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
#   * RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO or CMAKE_CXX_FLAGS_RELWITHDEBINFO
#   * MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL or CMAKE_CXX_FLAGS_MINSIZEREL) 
set(CMAKE_BUILD_TYPE Debug)

add_definitions(-std=c++0x)
SET(CMAKE_BUILD_TYPE Debug)

ADD_SUBDIRECTORY (gtest gtest)
enable_testing()

# Add CMake module path
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Add linear algebra 
add_subdirectory(rm_linear_algebra)

# Add quaternions
add_subdirectory(rm_quaternions)

# Add rotations
add_subdirectory(rm_rotations)

# Add Doxygen documentation
add_subdirectory(rm_doc)
