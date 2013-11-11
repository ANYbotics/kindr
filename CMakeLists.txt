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

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

add_subdirectory(rm_linear_algebra)

add_subdirectory(rm_rotations)

add_subdirectory(rm_doc)

