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

#include "kindr/poses/PoseDiff.hpp"
#include "kindr/poses/PoseDiffBase.hpp"
#include "kindr/phys_quant/PhysicalQuantities.hpp"

namespace pose = kindr;
namespace pos = kindr;
namespace rot = kindr;


typedef ::testing::Types<
    pose::TwistLinearVelocityRotationQuaternionDiffD,
    pose::TwistLinearVelocityRotationQuaternionDiffF
> Types;


template <typename PoseImplementation>
struct TwistTest: public ::testing::Test {
  typedef PoseImplementation PoseDiff;
};

TYPED_TEST_CASE(TwistTest, Types);

TYPED_TEST(TwistTest, testConstructor)
{

}

TYPED_TEST(TwistTest, testInitial)
{
  typedef typename TestFixture::PoseDiff PoseDiff;
  typedef typename TestFixture::PoseDiff::Scalar Scalar;
  typedef rot::RotationQuaternion<Scalar> Rotation;

  typename PoseDiff::PositionDiff linearVelocity(0.1, 0.2, 0.3);
  rot::LocalAngularVelocityD angularVelocity;
  typename PoseDiff::RotationDiff rquatDiff(1, 2, 3, 4);
  PoseDiff twist(linearVelocity, rquatDiff);
  std::cout << twist << std::endl;

  Rotation rot;
  std::cout << twist.getVector(rot) << std::endl;
}

TYPED_TEST(TwistTest, testSetZero)
{
  typedef typename TestFixture::PoseDiff PoseDiff;

  PoseDiff twist(typename PoseDiff::PositionDiff(0.1, 0.2, 0.3), typename PoseDiff::RotationDiff(1, 2, 3, 4));

  twist.setZero();

  ASSERT_EQ(twist.getTranslationalVelocity().vector().x(), 0);
  ASSERT_EQ(twist.getTranslationalVelocity().vector().y(), 0);
  ASSERT_EQ(twist.getTranslationalVelocity().vector().z(), 0);

  ASSERT_EQ(twist.getRotationalVelocity().w(), 0);
  ASSERT_EQ(twist.getRotationalVelocity().x(), 0);
  ASSERT_EQ(twist.getRotationalVelocity().y(), 0);
  ASSERT_EQ(twist.getRotationalVelocity().z(), 0);
}


