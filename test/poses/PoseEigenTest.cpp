/*
  /*
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#include <iostream>

#include <Eigen/Core>

#include <gtest/gtest.h>

#include "kindr/poses/PoseEigen.hpp"
#include "kindr/poses/PoseBase.hpp"
#include "kindr/phys_quant/PhysicalQuantitiesEigen.hpp"


namespace pose = kindr::poses::eigen_impl;
namespace pos = kindr::phys_quant::eigen_impl;
namespace rot = kindr::rotations::eigen_impl;


typedef ::testing::Types<
    pose::HomogeneousTransformationPosition3RotationQuaternionD,
    pose::HomogeneousTransformationPosition3RotationQuaternionF
> Types;


template <typename PoseImplementation>
struct HomogeneousTransformationTest: public ::testing::Test {
  typedef PoseImplementation Pose;
};

TYPED_TEST_CASE(HomogeneousTransformationTest, Types);

TYPED_TEST(HomogeneousTransformationTest, testConstructor)
{

}

TEST(PosesTest, testInitial)
{
  pose::HomogeneousTransformationPosition3RotationQuaternionD test;
  pose::HomogeneousTransformationPosition3RotationQuaternionD test2(pos::Position3D(1,2,3),rot::RotationQuaternionPD(rot::AngleAxisPD(0.5,1.0,0,0)));
  std::cout << "pos: " << test2.getPosition() << std::endl;
  std::cout << "rot: " << test2.getRotation() << std::endl;
  std::cout << "mat: " << test2.getTransformationMatrix()<< std::endl;
  rot::AngleAxisPD aa;
  aa = test2.getRotation();

  pose::HomogeneousTransformationPosition3RotationQuaternionD pose1(pos::Position3D(4,5,6),rot::RotationQuaternionPD(rot::AngleAxisPD(0.2,Eigen::Vector3d(4,2,5).normalized())));
  pos::Position3D posX(22,33,55);
  pos::Position3D posRes = pose1.transform(posX);
  std::cout << "posX: " << posX << std::endl;
  std::cout << "posRes: " << posRes << std::endl;
  std::cout << "posX again: " << pose1.inverseTransform(posRes) << std::endl;
  posRes = rot::AngleAxisPD(M_PI/2.0,Eigen::Vector3d(0,0,1).normalized()).rotate(pos::Position3D(1,0,0));
  std::cout << "posRes: " << posRes << std::endl;

  std::cout << "pose1: " << pose1 << std::endl;
}

TYPED_TEST(HomogeneousTransformationTest, testSetIdentity)
{
  pose::HomogeneousTransformationPosition3RotationQuaternionD test(pos::Position3D(1,2,3),rot::RotationQuaternionPD(rot::AngleAxisPD(0.5,1.0,0,0)));

  test.setIdentity();

  ASSERT_EQ(test.getPosition().vector().x(), 0);
  ASSERT_EQ(test.getPosition().vector().y(), 0);
  ASSERT_EQ(test.getPosition().vector().z(), 0);

  ASSERT_EQ(test.getRotation().w(), 1);
  ASSERT_EQ(test.getRotation().x(), 0);
  ASSERT_EQ(test.getRotation().y(), 0);
  ASSERT_EQ(test.getRotation().z(), 0);
}

TYPED_TEST(HomogeneousTransformationTest, testGenericRotateVectorCompilable)
{
  pose::HomogeneousTransformationPosition3RotationQuaternionD test(pos::Position3D(1,2,3),rot::RotationQuaternionPD(rot::AngleAxisPD(0.5,1.0,0,0)));

  pos::Velocity3D vel(-1,2,3);
  test.getRotation().rotate(vel);
}


