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

#include <kindr/Core>


typedef ::testing::Types<
    kindr::TwistLocalD,
    kindr::TwistLocalF,
    kindr::TwistGlobalD,
    kindr::TwistGlobalF
> TwistTypes;

template <typename Twist_>
struct TwistWithAngularVelocityTest: public ::testing::Test {
  typedef Twist_ Twist;
  typedef typename Twist::Scalar Scalar;
  typedef typename Twist::PositionDiff PositionDiff;
  typedef typename Twist::RotationDiff RotationDiff;
};

TYPED_TEST_CASE(TwistWithAngularVelocityTest, TwistTypes);

TYPED_TEST(TwistWithAngularVelocityTest, constructors)
{
  typedef typename TestFixture::Twist Twist;
  typedef typename TestFixture::PositionDiff PositionDiff;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;

  // Default constructor
  Twist twist1;
  EXPECT_EQ(0.0, twist1.getTranslationalVelocity().x());
  EXPECT_EQ(0.0, twist1.getTranslationalVelocity().y());
  EXPECT_EQ(0.0, twist1.getTranslationalVelocity().z());
  EXPECT_EQ(0.0, twist1.getRotationalVelocity().x());
  EXPECT_EQ(0.0, twist1.getRotationalVelocity().y());
  EXPECT_EQ(0.0, twist1.getRotationalVelocity().z());

  // Copy constructor
  Twist twist2(PositionDiff(1.0, 2.0, 3.0), RotationDiff(5.0, 6.0, 7.0));
  Twist twist3(twist2);
  EXPECT_EQ(1.0, twist3.getTranslationalVelocity().x());
  EXPECT_EQ(2.0, twist3.getTranslationalVelocity().y());
  EXPECT_EQ(3.0, twist3.getTranslationalVelocity().z());
  EXPECT_EQ(5.0, twist3.getRotationalVelocity().x());
  EXPECT_EQ(6.0, twist3.getRotationalVelocity().y());
  EXPECT_EQ(7.0, twist3.getRotationalVelocity().z());
  EXPECT_EQ(1.0, twist3.getVector()(0));
  EXPECT_EQ(2.0, twist3.getVector()(1));
  EXPECT_EQ(3.0, twist3.getVector()(2));
  EXPECT_EQ(5.0, twist3.getVector()(3));
  EXPECT_EQ(6.0, twist3.getVector()(4));
  EXPECT_EQ(7.0, twist3.getVector()(5));



  //  Vector6
  typename Twist::Vector6 vector6;
  vector6 << 1.0, 2.0, 3.0, 5.0, 6.0, 7.0;
  Twist twist4(vector6);
  EXPECT_EQ(1.0, twist4.getTranslationalVelocity().x());
  EXPECT_EQ(2.0, twist4.getTranslationalVelocity().y());
  EXPECT_EQ(3.0, twist4.getTranslationalVelocity().z());
  EXPECT_EQ(5.0, twist4.getRotationalVelocity().x());
  EXPECT_EQ(6.0, twist4.getRotationalVelocity().y());
  EXPECT_EQ(7.0, twist4.getRotationalVelocity().z());

  //  2xVector3
  typename Twist::Vector3 linVel(1.0, 2.0, 3.0);
  typename Twist::Vector3 angVel(5.0, 6.0, 7.0);
  Twist twist5(linVel, angVel);
  EXPECT_EQ(1.0, twist5.getTranslationalVelocity().x());
  EXPECT_EQ(2.0, twist5.getTranslationalVelocity().y());
  EXPECT_EQ(3.0, twist5.getTranslationalVelocity().z());
  EXPECT_EQ(5.0, twist5.getRotationalVelocity().x());
  EXPECT_EQ(6.0, twist5.getRotationalVelocity().y());
  EXPECT_EQ(7.0, twist5.getRotationalVelocity().z());

}


TYPED_TEST(TwistWithAngularVelocityTest, getters)
{
  typedef typename TestFixture::Twist Twist;
  typedef typename TestFixture::PositionDiff PositionDiff;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;

  Twist twist1(PositionDiff(1.0, 2.0, 3.0), RotationDiff(5.0, 6.0, 7.0));

  EXPECT_EQ(1.0, twist1.getTranslationalVelocity().x());
  EXPECT_EQ(2.0, twist1.getTranslationalVelocity().y());
  EXPECT_EQ(3.0, twist1.getTranslationalVelocity().z());
  EXPECT_EQ(5.0, twist1.getRotationalVelocity().x());
  EXPECT_EQ(6.0, twist1.getRotationalVelocity().y());
  EXPECT_EQ(7.0, twist1.getRotationalVelocity().z());
  EXPECT_EQ(1.0, twist1.getVector()(0));
  EXPECT_EQ(2.0, twist1.getVector()(1));
  EXPECT_EQ(3.0, twist1.getVector()(2));
  EXPECT_EQ(5.0, twist1.getVector()(3));
  EXPECT_EQ(6.0, twist1.getVector()(4));
  EXPECT_EQ(7.0, twist1.getVector()(5));

}

TYPED_TEST(TwistWithAngularVelocityTest, setters)
{
  typedef typename TestFixture::Twist Twist;
  typedef typename TestFixture::PositionDiff PositionDiff;
  typedef typename TestFixture::RotationDiff RotationDiff;
  typedef typename TestFixture::Scalar Scalar;

  Twist twist1;
  twist1.getTranslationalVelocity() = PositionDiff(1.0, 2.0, 3.0);
  twist1.getRotationalVelocity() = RotationDiff(5.0, 6.0, 7.0);
  EXPECT_EQ(1.0, twist1.getTranslationalVelocity().x());
  EXPECT_EQ(2.0, twist1.getTranslationalVelocity().y());
  EXPECT_EQ(3.0, twist1.getTranslationalVelocity().z());
  EXPECT_EQ(5.0, twist1.getRotationalVelocity().x());
  EXPECT_EQ(6.0, twist1.getRotationalVelocity().y());
  EXPECT_EQ(7.0, twist1.getRotationalVelocity().z());

  Twist twist2;
  typename Twist::Vector6 vector6;
  vector6 << 1.0, 2.0, 3.0, 5.0, 6.0, 7.0;
  twist2.setVector(vector6);
  EXPECT_EQ(1.0, twist2.getTranslationalVelocity().x());
  EXPECT_EQ(2.0, twist2.getTranslationalVelocity().y());
  EXPECT_EQ(3.0, twist2.getTranslationalVelocity().z());
  EXPECT_EQ(5.0, twist2.getRotationalVelocity().x());
  EXPECT_EQ(6.0, twist2.getRotationalVelocity().y());
  EXPECT_EQ(7.0, twist2.getRotationalVelocity().z());
}
