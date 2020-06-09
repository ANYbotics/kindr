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

#include "kindr/poses/Pose.hpp"
#include "kindr/poses/PoseBase.hpp"
#include "kindr/phys_quant/PhysicalQuantities.hpp"


typedef ::testing::Types<
    kindr::HomTransformQuatD,
    kindr::HomTransformQuatF,
    kindr::HomTransformMatrixD,
    kindr::HomTransformMatrixF
> Types;

template <typename PoseImplementation>
struct HomogeneousTransformationTest: public ::testing::Test {
  typedef PoseImplementation Pose;
  typedef typename Pose::Scalar Scalar;
  typedef typename Pose::Position Position;
  typedef typename Pose::Rotation Rotation;
};

TYPED_TEST_CASE(HomogeneousTransformationTest, Types);


TYPED_TEST(HomogeneousTransformationTest, testCopyConstructor)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;

  Pose poseA(Position(1.0,2.0,3.0), Rotation(kindr::EulerAnglesZyx<Scalar>(0.5,1.2,-1.7)));
  Pose poseB(poseA);

  ASSERT_EQ(poseA.getPosition().x(), poseB.getPosition().x());
  ASSERT_EQ(poseA.getPosition().y(), poseB.getPosition().y());
  ASSERT_EQ(poseA.getPosition().z(), poseB.getPosition().z());
  ASSERT_TRUE(poseA.getRotation().isNear(poseB.getRotation(),1.0e-6));
}


TYPED_TEST(HomogeneousTransformationTest, testSetIdentity)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;
  Pose test(Position(1,2,3), Rotation(kindr::AngleAxis<Scalar>(0.5,1.0,0,0)));

  test.setIdentity();

  ASSERT_EQ(test.getPosition().vector().x(), 0.0);
  ASSERT_EQ(test.getPosition().vector().y(), 0.0);
  ASSERT_EQ(test.getPosition().vector().z(), 0.0);

  ASSERT_TRUE(test.getRotation().isNear(Rotation(),1.0e-6));
}


TYPED_TEST(HomogeneousTransformationTest, testComparisonEqual)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;
  Position positionAToBInA(1.0,2.0,3.0);
  Position positionAToCInA(1.0,2.0,4.0);
  Rotation rotationBToA(kindr::EulerAnglesZyx<Scalar>(0.5, -0.9, 1.2));
  Rotation rotationCToA(kindr::EulerAnglesZyx<Scalar>(0.5, -0.9, 1.2));
  Pose poseBToA(positionAToBInA, rotationBToA);
  Pose poseBToA2(positionAToBInA, rotationBToA);
  Pose poseCToA(positionAToCInA, rotationCToA);

  // Check equality comparison
  ASSERT_TRUE(poseBToA == poseBToA2);
  ASSERT_FALSE(poseBToA == poseCToA);
}

TYPED_TEST(HomogeneousTransformationTest, testComparisonNotEqual)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;
  Position positionAToBInA(1.0,2.0,3.0);
  Position positionAToCInA(1.0,2.0,4.0);
  Rotation rotationBToA(kindr::EulerAnglesZyx<Scalar>(0.5, -0.9, 1.2));
  Rotation rotationCToA(kindr::EulerAnglesZyx<Scalar>(0.5, -0.9, 1.2));
  Pose poseBToA(positionAToBInA, rotationBToA);
  Pose poseBToA2(positionAToBInA, rotationBToA);
  Pose poseCToA(positionAToCInA, rotationCToA);

  // Check inequality comparison
  ASSERT_FALSE(poseBToA != poseBToA2);
  ASSERT_TRUE(poseBToA != poseCToA);
}


TYPED_TEST(HomogeneousTransformationTest, testTransform)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;
  Position positionAToBInA(1.0,2.0,3.0);
  Rotation rotationBToA(kindr::EulerAnglesZyx<Scalar>(0.5, -0.9, 1.2));
  Pose poseBToA(positionAToBInA, rotationBToA);
  Position positionInB(0.5, 0.4, -5.4);
  Position positionInA = poseBToA.transform(positionInB);
  Position expectedPositionInA = rotationBToA.rotate(positionInB) + positionAToBInA;

  EXPECT_NEAR(expectedPositionInA.x(), positionInA.x(), 1.0e-6);
  EXPECT_NEAR(expectedPositionInA.y(), positionInA.y(), 1.0e-6);
  EXPECT_NEAR(expectedPositionInA.z(), positionInA.z(), 1.0e-6);
}

TYPED_TEST(HomogeneousTransformationTest, testInverseTransform)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;
  Position positionAToBInA(1.0,2.0,3.0);
  Rotation rotationBToA(kindr::EulerAnglesZyx<Scalar>(0.5, -0.9, 1.2));
  Pose poseBToA(positionAToBInA, rotationBToA);
  Position positionInA(0.5, 0.4, -5.4);
  Position positionInB = poseBToA.inverseTransform(positionInA);
  Position expectedPositionInB = rotationBToA.inverseRotate(positionInA - positionAToBInA);

  EXPECT_NEAR(expectedPositionInB.x(), positionInB.x(), 1.0e-6);
  EXPECT_NEAR(expectedPositionInB.y(), positionInB.y(), 1.0e-6);
  EXPECT_NEAR(expectedPositionInB.z(), positionInB.z(), 1.0e-6);
}


TYPED_TEST(HomogeneousTransformationTest, testTransformAndInverseTransform)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;
  Position positionAToBInA(1.0,2.0,3.0);
  Rotation rotationBToA(kindr::EulerAnglesZyx<Scalar>(0.5, -0.9, 1.2));
  Pose poseBToA(positionAToBInA, rotationBToA);
  Position positionInB(0.5, 0.4, -5.4);
  Position positionInA = poseBToA.transform(positionInB);
  Position positionInBNew = poseBToA.inverseTransform(positionInA);
  EXPECT_NEAR(positionInB.x(), positionInBNew.x(), 1.0e-6);
  EXPECT_NEAR(positionInB.y(), positionInBNew.y(), 1.0e-6);
  EXPECT_NEAR(positionInB.z(), positionInBNew.z(), 1.0e-6);
}

TYPED_TEST(HomogeneousTransformationTest, testConcatenation)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;
  Position positionAToBInA(0.5, -0.8, 1.3);
  Rotation rotationBToA(kindr::EulerAnglesZyx<Scalar>(0.4, 0.6, -2.5));
  Pose poseBToA(positionAToBInA, rotationBToA);

  Position positionBToCInB(5.7, 1.3, -5.1);
  Rotation rotationCToB(kindr::EulerAnglesZyx<Scalar>(-0.9, 2.1, 7.9));
  Pose poseCToB(positionBToCInB, rotationCToB);

  Position positionInC(-0.8, 5.0, 1.6);

  Pose poseCToA = poseBToA*poseCToB;

  Position positionConcatTransform = poseCToA.transform(positionInC);
  Position positionTransform = (rotationBToA*rotationCToB).rotate(positionInC) +  (positionAToBInA + rotationBToA.rotate(positionBToCInB));
  EXPECT_NEAR(positionTransform.x(), positionConcatTransform.x(), 1.0e-3);
  EXPECT_NEAR(positionTransform.y(), positionConcatTransform.y(), 1.0e-3);
  EXPECT_NEAR(positionTransform.z(), positionConcatTransform.z(), 1.0e-3);
}


TYPED_TEST(HomogeneousTransformationTest, testGenericRotateVectorCompilable)
{
  typedef typename TestFixture::Pose Pose;
  typedef typename TestFixture::Position Position;
  typedef typename TestFixture::Rotation Rotation;
  typedef typename TestFixture::Scalar Scalar;

  Pose test(Position(1,2,3),Rotation(kindr::AngleAxis<Scalar>(0.5,1.0,0,0)));

  kindr::Velocity<Scalar, 3> vel(-1,2,3);
  test.getRotation().rotate(vel);
}
