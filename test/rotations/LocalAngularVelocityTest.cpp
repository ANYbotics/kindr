/* Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
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

#include "kindr/rotations/RotationDiff.hpp"
#include "kindr/common/gtest_eigen.hpp"

namespace rot = kindr;

template <typename Implementation>
struct LocalAngularVelocityTest: public ::testing::Test {
  typedef Implementation LocalAngularVelocity;
  typedef typename LocalAngularVelocity::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;


  Vector3 eigenVector3vZero = Vector3::Zero();
  Vector3 eigenVector3v1 = Vector3(1.1, 2.2, 3.3);
  Vector3 eigenVector3v2 = Vector3(10,20,30);
  Vector3 eigenVector3v3 = Vector3(1,2,3);
  Vector3 eigenVector3vAdd2And3 = Vector3(11,22,33);
  Vector3 eigenVector3vSubtract3from2 = Vector3(9,18,27);



  LocalAngularVelocityTest() {}
};





typedef ::testing::Types<
    rot::LocalAngularVelocityPD,
    rot::LocalAngularVelocityPF
> LocalAngularVelocityTypes;

TYPED_TEST_CASE(LocalAngularVelocityTest, LocalAngularVelocityTypes);



TYPED_TEST(LocalAngularVelocityTest, testConstructors)
{
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;

  // default constructor
  LocalAngularVelocity angVel1;
  ASSERT_EQ(this->eigenVector3vZero.x(), angVel1.x());
  ASSERT_EQ(this->eigenVector3vZero.y(), angVel1.y());
  ASSERT_EQ(this->eigenVector3vZero.z(), angVel1.z());

  // constructor with three values (x,y,z)
  LocalAngularVelocity angVel2(this->eigenVector3v1(0),this->eigenVector3v1(1),this->eigenVector3v1(2));
  ASSERT_EQ(this->eigenVector3v1.x(), angVel2.x());
  ASSERT_EQ(this->eigenVector3v1.y(), angVel2.y());
  ASSERT_EQ(this->eigenVector3v1.z(), angVel2.z());

  // constructor with Eigen vector
  LocalAngularVelocity angVel3(this->eigenVector3v1);
  ASSERT_EQ(this->eigenVector3v1.x(), angVel3.x());
  ASSERT_EQ(this->eigenVector3v1.y(), angVel3.y());
  ASSERT_EQ(this->eigenVector3v1.z(), angVel3.z());

  // constructor with LocalAngularVelocity
  LocalAngularVelocity angVel4(angVel3);
  ASSERT_EQ(this->eigenVector3v1.x(), angVel4.x());
  ASSERT_EQ(this->eigenVector3v1.y(), angVel4.y());
  ASSERT_EQ(this->eigenVector3v1.z(), angVel4.z());

}

TYPED_TEST(LocalAngularVelocityTest, testGetters)
{
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;

  // toImplementation
  LocalAngularVelocity angVel3(this->eigenVector3v1);
  ASSERT_EQ(this->eigenVector3v1.x(), angVel3.toImplementation()(0));
  ASSERT_EQ(this->eigenVector3v1.y(), angVel3.toImplementation()(1));
  ASSERT_EQ(this->eigenVector3v1.z(), angVel3.toImplementation()(2));


  LocalAngularVelocity angVel2;
  angVel2.x() = this->eigenVector3v1.x();
  angVel2.y() = this->eigenVector3v1.y();
  angVel2.z() = this->eigenVector3v1.z();
  ASSERT_EQ(this->eigenVector3v1.x(), angVel2.x());
  ASSERT_EQ(this->eigenVector3v1.y(), angVel2.y());
  ASSERT_EQ(this->eigenVector3v1.z(), angVel2.z());
}

TYPED_TEST(LocalAngularVelocityTest, testSetters)
{
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;
  // setZero()
  LocalAngularVelocity angVel1(this->eigenVector3v1(0),this->eigenVector3v1(1),this->eigenVector3v1(2));
  angVel1.setZero();
  ASSERT_EQ(this->eigenVector3vZero.x(), angVel1.x());
  ASSERT_EQ(this->eigenVector3vZero.y(), angVel1.y());
  ASSERT_EQ(this->eigenVector3vZero.z(), angVel1.z());
}

TYPED_TEST(LocalAngularVelocityTest, testAddition)
{
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;

  const LocalAngularVelocity angVel2(this->eigenVector3v2);
  const LocalAngularVelocity angVel3(this->eigenVector3v3);

  // addition
  LocalAngularVelocity velAdd = angVel2+angVel3;
  ASSERT_EQ(this->eigenVector3vAdd2And3.x(), velAdd.x());
  ASSERT_EQ(this->eigenVector3vAdd2And3.y(), velAdd.y());
  ASSERT_EQ(this->eigenVector3vAdd2And3.z(), velAdd.z());

  // addition and assignment
  LocalAngularVelocity velAddandAssign(this->eigenVector3v2);
  velAddandAssign += angVel3;
  ASSERT_EQ(this->eigenVector3vAdd2And3.x(), velAddandAssign.x());
  ASSERT_EQ(this->eigenVector3vAdd2And3.y(), velAddandAssign.y());
  ASSERT_EQ(this->eigenVector3vAdd2And3.z(), velAddandAssign.z());

  // subtract
  LocalAngularVelocity velSubtract = angVel2-angVel3;
  ASSERT_EQ(this->eigenVector3vSubtract3from2.x(), velSubtract.x());
  ASSERT_EQ(this->eigenVector3vSubtract3from2.y(), velSubtract.y());
  ASSERT_EQ(this->eigenVector3vSubtract3from2.z(), velSubtract.z());

  // subtract and assignment
  LocalAngularVelocity velSubtractandAssign(this->eigenVector3v2);
  velSubtractandAssign -= angVel3;
  ASSERT_EQ(this->eigenVector3vSubtract3from2.x(), velSubtractandAssign.x());
  ASSERT_EQ(this->eigenVector3vSubtract3from2.y(), velSubtractandAssign.y());
  ASSERT_EQ(this->eigenVector3vSubtract3from2.z(), velSubtractandAssign.z());
}

TYPED_TEST(LocalAngularVelocityTest, testMultiplication)
{
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;

  const LocalAngularVelocity angVel2(this->eigenVector3v2);
  const LocalAngularVelocity angVel3(this->eigenVector3v3);

  // multiplication 1
  LocalAngularVelocity mult1 = angVel3 * 10.0;
  ASSERT_EQ(angVel2.x(), mult1.x());
  ASSERT_EQ(angVel2.y(), mult1.y());
  ASSERT_EQ(angVel2.z(), mult1.z());

  // multiplication 2
  LocalAngularVelocity mult2 = 10.0 * angVel3;
  ASSERT_EQ(angVel2.x(), mult2.x());
  ASSERT_EQ(angVel2.y(), mult2.y());
  ASSERT_EQ(angVel2.z(), mult2.z());
}

TYPED_TEST(LocalAngularVelocityTest, testConversionFromGlobalAngularVelocity)
{
  typedef typename TestFixture::LocalAngularVelocity LocalAngularVelocity;
  typedef typename TestFixture::Scalar Scalar;
  using namespace kindr;

  Eigen::Matrix<Scalar, 3, 1> globalVector(1.0, 2.0, 3.0);
  GlobalAngularVelocity<Scalar> globalAngularVelocity(globalVector);
  RotationQuaternion<Scalar> rotationLocalToGlobal;
  rotationLocalToGlobal.setRandom();
  LocalAngularVelocity localAngularVelocity(rotationLocalToGlobal, globalAngularVelocity);
  Eigen::Matrix<Scalar, 3, 1> localVector = rotationLocalToGlobal.inverseRotate(globalVector);
  ASSERT_EQ(localVector.x(), localAngularVelocity.x());
  ASSERT_EQ(localVector.y(), localAngularVelocity.y());
  ASSERT_EQ(localVector.z(), localAngularVelocity.z());
}


