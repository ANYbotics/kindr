# Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
# Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
# OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
find_package(ROS REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(../include)

 
if (BUILD_GTEST)
################################
# Unit Tests
################################
################################
# GTest
################################

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(test)

# This macro globs all tests to a list so we can run them post-build.
macro(add_gtest target)
  set(sources  ${ARGV})
  LIST(REMOVE_ITEM sources ${target})
  add_executable(${target} ${sources})
  message("Building test for " ${sources})
  set(UNIT_TEST_TARGETS ${UNIT_TEST_TARGETS} ${target})
  target_link_libraries(${target} gtest gtest_main pthread)
  add_test(${target} ${target})
endmacro(add_gtest)

set(LINEARALGEBRA_SRCS
      test_main.cpp 
      linear_algebra/SkewMatrixFromVectorTest.cpp
)
add_gtest(runUnitTestsLinearAlgebra ${LINEARALGEBRA_SRCS})

set(QUATERNIONS_SRCS
	test_main.cpp
	quaternions/QuaternionEigenTest.cpp
)
add_gtest( runUnitTestsQuaternions ${QUATERNIONS_SRCS} )

set(ROTATION_SRCS
	test_main.cpp
	rotations/eigen/RotationMatrixTest.cpp
	rotations/eigen/RotationQuaternionTest.cpp
	rotations/eigen/RotationVectorTest.cpp
	rotations/eigen/AngleAxisTest.cpp
	rotations/eigen/RotationMatrixTest.cpp
	rotations/eigen/EulerAnglesZyxTest.cpp
	rotations/eigen/EulerAnglesXyzTest.cpp
	rotations/RotationEigenTest.cpp
)
add_gtest( runUnitTestsRotation ${ROTATION_SRCS})

set(ROTATIONQUATERNION_SRCS
	test_main.cpp
	rotations/eigen/RotationQuaternionTest.cpp
)
add_gtest( runUnitTestsRotationQuaternion ${ROTATIONQUATERNION_SRCS})


set(ROTATIONDIFF_SRCS
	test_main.cpp
	rotations/RotationDiffEigenTest.cpp
	rotations/eigen/LocalAngularVelocityTest.cpp
	rotations/eigen/RotationQuaternionDiffTest.cpp
	rotations/eigen/RotationMatrixDiffTest.cpp
	rotations/eigen/EulerAnglesZyxDiffTest.cpp
	rotations/eigen/EulerAnglesXyzDiffTest.cpp
)
add_gtest( runUnitTestsRotationDiff ${ROTATIONDIFF_SRCS})

set(POSITIONS_SRCS
	test_main.cpp
	positions/PositionEigenTest.cpp
)
add_gtest( runUnitTestsPositions  ${POSITIONS_SRCS})

set(POSITIONSDIFF_SRCS
	test_main.cpp
	positions/PositionDiffEigenTest.cpp
)
add_gtest( runUnitTestsPositionsDiff  ${POSITIONSDIFF_SRCS})


set(POSES_SRCS
	test_main.cpp
	poses/PoseEigenTest.cpp
)
add_gtest( runUnitTestsPose  ${POSES_SRCS})

set(POSESDIFF_SRCS
	test_main.cpp
	poses/PoseDiffEigenTest.cpp
)
add_gtest( runUnitTestsPoseDiff  ${POSESDIFF_SRCS})

set(VECTOR_SRCS
	test_main.cpp
	vectors/VectorsTest.cpp
)
add_gtest( runUnitTestsVector  ${VECTOR_SRCS})

set(FORCE_SRCS
	test_main.cpp
	phys_quant/ForceTest.cpp
)
add_gtest( runUnitTestsForce  ${FORCE_SRCS})

IF (ROS_FOUND)
    set(THIRDPARTY_ROS_SRCS
    	test_main.cpp
    	thirdparty/ros/RosTfPoseEigenTest.cpp
    	thirdparty/ros/RosGeometryMsgPoseEigenTest.cpp
    	thirdparty/ros/RosGeometryMsgRotationEigenTest.cpp
    	thirdparty/ros/RosGeometryMsgPhysicalQuantitiesEigenTest.cpp
    )
    add_gtest( runUnitTestsThirdpartyRos  ${THIRDPARTY_ROS_SRCS})
ENDIF (ROS_FOUND)

# Run all unit tests post-build.
add_custom_target(run_tests ALL
                  DEPENDS ${UNIT_TEST_TARGETS}
)
add_custom_command(TARGET run_tests
                   COMMENT "Running tests"
                   POST_BUILD COMMAND ${PROJECT_SOURCE_DIR}/test/run_tests.py ${CMAKE_BINARY_DIR} ${UNIT_TEST_TARGETS}
                   WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
)
endif()
