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

#include <gtest/gtest.h>

#include <rm/common/assert_macros_eigen.hpp>
#include <rm/common/gtest_eigen.hpp>

#include <rm/quaternions/QuaternionEigen.hpp>

namespace quat = rm::quaternions::eigen_implementation;

typedef ::testing::Types<
    std::pair<quat::QuaternionD, quat::UnitQuaternionD>,
    std::pair<quat::QuaternionF, quat::UnitQuaternionF>
> TypePairs;


template <typename QuaternionImplementationPair>
struct QuaternionsPairsTest : public ::testing::Test  {

  typedef typename QuaternionImplementationPair::first_type Quaternion;
  typedef typename QuaternionImplementationPair::second_type UnitQuaternion;

  typedef typename Quaternion::Scalar QuaternionScalar;
  typedef typename UnitQuaternion::Scalar UnitQuaternionScalar;
  typedef Eigen::Matrix<QuaternionScalar, 3, 1> Vector3;
  typedef Eigen::Matrix<QuaternionScalar, 3, 3> Matrix3x4;

  Quaternion quatZero =     Quaternion(Eigen::Quaterniond(0, 0, 0, 0).cast<QuaternionScalar>());
  Quaternion quatIdentity = Quaternion(Eigen::Quaterniond(1, 0, 0, 0).cast<QuaternionScalar>());
  UnitQuaternion uquatIdentity = UnitQuaternion(quat::Quaternion<QuaternionScalar>(Eigen::Quaterniond(1, 0, 0, 0).cast<QuaternionScalar>()));
  Quaternion quatGeneric =  Quaternion(quat::Quaternion<QuaternionScalar>(Eigen::Quaterniond(1, 2, 3, 4).cast<QuaternionScalar>()));
  Quaternion quatGenericInverse =  Quaternion(quat::Quaternion<QuaternionScalar>(Eigen::Quaterniond(   0.033333333333333, -0.066666666666667, -0.1, -0.133333333333333).cast<QuaternionScalar>()));
  UnitQuaternion uquatGeneric =  UnitQuaternion(quat::UnitQuaternion<UnitQuaternionScalar>(Eigen::Quaterniond(1.0/sqrt(1.0+2.0*2.0+3.0*3.0+4.0*4.0), 2.0/sqrt(1.0+2.0*2.0+3.0*3.0+4.0*4.0), 3.0/sqrt(1.0+2.0*2.0+3.0*3.0+4.0*4.0), 4.0/sqrt(1.0+2.0*2.0+3.0*3.0+4.0*4.0)).cast<UnitQuaternionScalar>()));
  UnitQuaternion uquatGenericInverse =  UnitQuaternion(quat::UnitQuaternion<UnitQuaternionScalar>(Eigen::Quaterniond(1.0/sqrt(1.0+2.0*2.0+3.0*3.0+4.0*4.0), -2.0/sqrt(1.0+2.0*2.0+3.0*3.0+4.0*4.0), -3.0/sqrt(1.0+2.0*2.0+3.0*3.0+4.0*4.0), -4.0/sqrt(1.0+2.0*2.0+3.0*3.0+4.0*4.0)).cast<UnitQuaternionScalar>()));

  QuaternionsPairsTest() {}
};

TYPED_TEST_CASE(QuaternionsPairsTest, TypePairs);

TYPED_TEST (QuaternionsPairsTest, testQuaternionConstructor ) {

  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::Quaternion::Scalar QuaternionScalar;
  typedef typename TestFixture::UnitQuaternion::Scalar UnitQuaternionScalar;


  // default constructor of quaternion needs to set all coefficients to zero
  Quaternion quat;
  ASSERT_EQ(quat.w(),QuaternionScalar(1));
  ASSERT_EQ(quat.x(),QuaternionScalar(0));
  ASSERT_EQ(quat.y(),QuaternionScalar(0));
  ASSERT_EQ(quat.z(),QuaternionScalar(0));

  // default constructor of unit quaternion needs to set the scalar to 1 and all other coefficients to zero
  UnitQuaternion uquat;
  ASSERT_EQ(uquat.w(),QuaternionScalar(1));
  ASSERT_EQ(uquat.x(),QuaternionScalar(0));
  ASSERT_EQ(uquat.y(),QuaternionScalar(0));
  ASSERT_EQ(uquat.z(),QuaternionScalar(0));

  // constructor of quaternion
  Quaternion quat1(this->quatGeneric.w(), this->quatGeneric.x(), this->quatGeneric.y(), this->quatGeneric.z());
  ASSERT_EQ(quat1.w(),this->quatGeneric.w());
  ASSERT_EQ(quat1.x(),this->quatGeneric.x());
  ASSERT_EQ(quat1.y(),this->quatGeneric.y());
  ASSERT_EQ(quat1.z(),this->quatGeneric.z());

  // constructor of quaternion
  UnitQuaternion uquat1(this->uquatGeneric.w(), this->uquatGeneric.x(), this->uquatGeneric.y(), this->uquatGeneric.z());
  ASSERT_EQ(uquat1.w(),this->uquatGeneric.w());
  ASSERT_EQ(uquat1.x(),this->uquatGeneric.x());
  ASSERT_EQ(uquat1.y(),this->uquatGeneric.y());
  ASSERT_EQ(uquat1.z(),this->uquatGeneric.z());
}



TYPED_TEST (QuaternionsPairsTest, testQuaternionInversion ) {
  typedef typename TestFixture::Quaternion Quaternion;
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::Quaternion::Scalar QuaternionScalar;
  typedef typename TestFixture::UnitQuaternion::Scalar UnitQuaternionScalar;

  // inverse of quaternion
  Quaternion invquat = this->quatGeneric.inverted();
  ASSERT_NEAR(invquat.w(), this->quatGenericInverse.w(),1e-6);
  ASSERT_NEAR(invquat.x(), this->quatGenericInverse.x(),1e-6);
  ASSERT_NEAR(invquat.y(), this->quatGenericInverse.y(),1e-6);
  ASSERT_NEAR(invquat.z(), this->quatGenericInverse.z(),1e-6);

  Quaternion quat = this->quatGeneric;
  quat.invert();
  ASSERT_NEAR(quat.w(), this->quatGenericInverse.w(),1e-6);
  ASSERT_NEAR(quat.x(), this->quatGenericInverse.x(),1e-6);
  ASSERT_NEAR(quat.y(), this->quatGenericInverse.y(),1e-6);
  ASSERT_NEAR(quat.z(), this->quatGenericInverse.z(),1e-6);

  // inverse of unit quaternion
  UnitQuaternion invuquat = this->uquatGeneric.inverted();
  ASSERT_NEAR(invuquat.w(), this->uquatGenericInverse.w(),1e-6);
  ASSERT_NEAR(invuquat.x(), this->uquatGenericInverse.x(),1e-6);
  ASSERT_NEAR(invuquat.y(), this->uquatGenericInverse.y(),1e-6);
  ASSERT_NEAR(invuquat.z(), this->uquatGenericInverse.z(),1e-6);

  UnitQuaternion uquat = this->uquatGeneric;
  uquat.invert();
  ASSERT_NEAR(uquat.w(), this->uquatGenericInverse.w(),1e-6);
  ASSERT_NEAR(uquat.x(), this->uquatGenericInverse.x(),1e-6);
  ASSERT_NEAR(uquat.y(), this->uquatGenericInverse.y(),1e-6);
  ASSERT_NEAR(uquat.z(), this->uquatGenericInverse.z(),1e-6);

}
