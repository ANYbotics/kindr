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

add_definitions(-std=c++11)

find_package(Eigen REQUIRED)

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

set(COMMON_SRCS
      test_main.cpp 
      common/CommonTest.cpp
)
add_gtest(runUnitTestsCommon ${COMMON_SRCS})


set(LINEARALGEBRA_SRCS
      test_main.cpp 
      linear_algebra/SkewMatrixFromVectorTest.cpp
			linear_algebra/PseudoInverseTest.cpp
	)
add_gtest(runUnitTestsLinearAlgebra ${LINEARALGEBRA_SRCS})

set(QUATERNIONS_SRCS
	test_main.cpp
	quaternions/QuaternionTest.cpp
)
add_gtest( runUnitTestsQuaternions ${QUATERNIONS_SRCS} )

set(ROTATION_SRCS
	test_main.cpp
	rotations/RotationMatrixTest.cpp
	rotations/RotationQuaternionTest.cpp
	rotations/RotationVectorTest.cpp
	rotations/AngleAxisTest.cpp
	rotations/RotationMatrixTest.cpp
	rotations/EulerAnglesZyxTest.cpp
	rotations/EulerAnglesXyzTest.cpp
	rotations/RotationTest.cpp
	rotations/ConventionTest.cpp

)
add_gtest( runUnitTestsRotation ${ROTATION_SRCS})


set(ROTATIONDIFF_SRCS
	test_main.cpp
	rotations/RotationDiffTest.cpp
	rotations/LocalAngularVelocityTest.cpp
	rotations/GlobalAngularVelocityTest.cpp
	rotations/RotationQuaternionDiffTest.cpp
	rotations/RotationMatrixDiffTest.cpp
	rotations/EulerAnglesZyxDiffTest.cpp
	rotations/EulerAnglesXyzDiffTest.cpp
)
add_gtest( runUnitTestsRotationDiff ${ROTATIONDIFF_SRCS})

set(POSITIONS_SRCS
	test_main.cpp

)
add_gtest( runUnitTestsPositions  ${POSITIONS_SRCS})

set(POSES_SRCS
	test_main.cpp
	poses/PositionTest.cpp
	poses/HomogeneousTransformationTest.cpp
)
add_gtest( runUnitTestsPose  ${POSES_SRCS})

set(POSESDIFF_SRCS
	test_main.cpp
	poses/PoseDiffTest.cpp
	poses/PositionDiffTest.cpp
	poses/TwistWithAngularVelocityTest.cpp
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
	phys_quant/WrenchTest.cpp
)
add_gtest( runUnitTestsForce  ${FORCE_SRCS})

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
