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

include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(../include)
#include_directories(../common/include)
#include_directories(../quaternions/include)
#include_directories(../positions/include)
#include_directories(../linear_algebra/include)

#add_executable( rot_example EXCLUDE_FROM_ALL ./src/example.cpp)
#ADD_CUSTOM_TARGET(demo_rotations rot_example DEPENDS rot_example) 

 
if (BUILD_GTEST)
################################
# Unit Tests
################################
################################
# GTest
################################

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(test)

set(LINEARALGEBRA_SRCS
      test_main.cpp 
      linear_algebra/SkewMatrixFromVectorTest.cpp
)

# Add test cpp file
add_executable( runUnitTestsLinearAlgebra  ${LINEARALGEBRA_SRCS})
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTestsLinearAlgebra gtest gtest_main)
add_test( runUnitTestsLinearAlgebra runUnitTestsLinearAlgebra )


set(QUATERNIONS_SRCS
	test_main.cpp
	quaternions/QuaternionEigenTest.cpp
)

# Add test cpp file
add_executable( runUnitTestsQuaternions  ${QUATERNIONS_SRCS})
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTestsQuaternions  gtest_main gtest pthread )
add_test( runUnitTestsQuaternions runUnitTestsQuaternions )



set(ROTATION_SRCS
	test_main.cpp
	rotations/eigen/RotationQuaternionTest.cpp
	rotations/eigen/RotationVectorTest.cpp
	rotations/eigen/AngleAxisTest.cpp
	rotations/eigen/RotationMatrixTest.cpp
	rotations/eigen/EulerAnglesZyxTest.cpp
	rotations/RotationEigenTest.cpp
)

# Add test cpp file
add_executable( runUnitTestsRotation ${ROTATION_SRCS})
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTestsRotation gtest_main gtest pthread  )
add_test(runUnitTestsRotation runUnitTestsRotation )



set(ROTATIONDIFF_SRCS
	test_main.cpp
	rotations/RotationDiffEigenTest.cpp
	rotations/eigen/LocalAngularVelocityTest.cpp
	rotations/eigen/AngleAxisDiffTest.cpp
	rotations/eigen/RotationVectorDiffTest.cpp
	rotations/eigen/RotationQuaternionDiffTest.cpp
	rotations/eigen/RotationMatrixDiffTest.cpp
	rotations/eigen/EulerAnglesZyxDiffTest.cpp
	rotations/eigen/EulerAnglesXyzDiffTest.cpp

)

# Add test cpp file
#add_executable( runUnitTestsRotationDiff ${ROTATIONDIFF_SRCS})
# Link test executable against gtest & gtest_main
#target_link_libraries(runUnitTestsRotationDiff gtest_main gtest pthread  )
#add_test(runUnitTestsRotationDiff runUnitTestsRotationDiff )

set(POSITIONS_SRCS
	test_main.cpp
	positions/PositionEigenTest.cpp
	positions/PositionDiffEigenTest.cpp
)

# Add test cpp file
add_executable( runUnitTestsPositions  ${POSITIONS_SRCS})
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTestsPositions  gtest_main gtest pthread )
add_test( runUnitTestsQuaternions runUnitTestsQuaternions )


set(POSES_SRCS
	test_main.cpp
	poses/PoseEigenTest.cpp
)

# Add test cpp file
#add_executable( runUnitTestsPose  ${POSES_SRCS})
# Link test executable against gtest & gtest_main
#target_link_libraries(runUnitTestsPose  gtest_main gtest pthread )
#add_test( runUnitTestsPose runUnitTestsPose )

set(POSESDIFF_SRCS
	test_main.cpp
	poses/PoseDiffEigenTest.cpp
)

# Add test cpp file
#add_executable( runUnitTestsPoseDiff  ${POSESDIFF_SRCS})
# Link test executable against gtest & gtest_main
#target_link_libraries(runUnitTestsPoseDiff  gtest_main gtest pthread )
#add_test( runUnitTestsPoseDiff runUnitTestsPoseDiff )

endif()