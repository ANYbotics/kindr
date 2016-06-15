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
project(kindr)

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED)

catkin_package(
    INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
    DEPENDS eigen    
)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

#################
## Documenting ##
#################
# Add Doxygen documentation
add_subdirectory(doc/doxygen)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}_rotations
    test/test_main.cpp
	#test/rotations/RotationMatrixTest.cpp
	#test/rotations/RotationQuaternionTest.cpp
	#test/rotations/RotationVectorTest.cpp
	#test/rotations/AngleAxisTest.cpp
    test/rotations/EulerAnglesZyxTest.cpp
#	test/rotations/EulerAnglesXyzTest.cpp
	test/rotations/RotationTest.cpp
	
#    test/rotations/LocalAngularVelocityTest.cpp
#	test/rotations/RotationQuaternionDiffTest.cpp
#	test/rotations/RotationMatrixDiffTest.cpp
test/rotations/EulerAnglesZyxDiffTest.cpp
test/rotations/EulerAnglesXyzDiffTest.cpp
#   test/rotations/RotationDiffTest.cpp
  )
  if(TARGET test_${PROJECT_NAME}_rotations)
    target_link_libraries(test_${PROJECT_NAME}_rotations ${catkin_LIBRARIES})
  endif()
#  
#  catkin_add_gtest(test_${PROJECT_NAME}_quaternions
#    test/test_main.cpp
#	test/quaternions/QuaternionEigenTest.cpp
#  )
#  if(TARGET test_${PROJECT_NAME}_quaternions)
#    target_link_libraries(test_${PROJECT_NAME}_quaternions ${catkin_LIBRARIES})
#  endif()
#  
#  catkin_add_gtest(test_${PROJECT_NAME}_math
#    test/test_main.cpp
#	test/linear_algebra/SkewMatrixFromVectorTest.cpp
#  )
#  if(TARGET test_${PROJECT_NAME}_math)
#    target_link_libraries(test_${PROJECT_NAME}_math ${catkin_LIBRARIES})
#  endif()
#  
#  catkin_add_gtest(test_${PROJECT_NAME}_poses
#    test/test_main.cpp
#	test/poses/PositionTest.cpp
#	test/poses/PositionDiffTest.cpp
#    test/poses/PoseTest.cpp
#	test/poses/PoseDiffTest.cpp
#  )
#  if(TARGET test_${PROJECT_NAME}_poses)
#    target_link_libraries(test_${PROJECT_NAME}_poses ${catkin_LIBRARIES})
#  endif()
#	
#  catkin_add_gtest(test_${PROJECT_NAME}_vectors
#    test/test_main.cpp
#	test/vectors/VectorsTest.cpp
#  )
#  if(TARGET test_${PROJECT_NAME}_vectors)
#    target_link_libraries(test_${PROJECT_NAME}_vectors ${catkin_LIBRARIES})
#  endif()
#	
#  catkin_add_gtest(test_${PROJECT_NAME}_phys_quant
#    test/test_main.cpp
#	test/phys_quant/ForceTest.cpp
#  )
#  if(TARGET test_${PROJECT_NAME}_phys_quant)
#    target_link_libraries(test_${PROJECT_NAME}_phys_quant ${catkin_LIBRARIES})
#  endif()
#
#  catkin_add_gtest(test_${PROJECT_NAME}_ros
#	test/test_main.cpp
#	test/thirdparty/ros/RosTfPoseEigenTest.cpp
#	test/thirdparty/ros/RosGeometryMsgPoseEigenTest.cpp
#	test/thirdparty/ros/RosGeometryMsgRotationEigenTest.cpp
#	test/thirdparty/ros/RosGeometryMsgPhysicalQuantitiesEigenTest.cpp
#  )
#  if(TARGET test_${PROJECT_NAME}_ros)
#    target_link_libraries(test_${PROJECT_NAME}_ros ${catkin_LIBRARIES})
#  endif()
endif()

