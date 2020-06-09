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

#include "kindr/rotations/Rotation.hpp"
#include "kindr/common/gtest_eigen.hpp"
#include "kindr/quaternions/Quaternion.hpp"
#include <cmath>

namespace rot = kindr;
namespace quat = kindr;

using namespace kindr;

template <typename RotationQuaternionImplementation>
class RotationQuaternionSingleTest : public ::testing::Test{
 public:
  typedef RotationQuaternionImplementation RotationQuaternion;
  typedef typename RotationQuaternionImplementation::Scalar Scalar;
  typedef typename RotationQuaternionImplementation::Implementation EigenQuat;
  typedef typename quat::UnitQuaternion<Scalar> UnitQuaternion;
  typedef Eigen::Matrix<Scalar,3,1> Vector;

  const EigenQuat eigenQuat1 = EigenQuat(0.0,0.36,0.48,0.8);
  const EigenQuat eigenQuat2 = EigenQuat(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));

  const EigenQuat eigenQuat1Conj = EigenQuat(0.0,-0.36,-0.48,-0.8);
  const EigenQuat eigenQuatIdentity = EigenQuat(1.0,0.0,0.0,0.0);
  const UnitQuaternion quat1 = UnitQuaternion(eigenQuat1);
  const UnitQuaternion quat2 = UnitQuaternion(eigenQuat2);
  const UnitQuaternion quatIdentity = UnitQuaternion(eigenQuatIdentity);
  const RotationQuaternion rotQuat1 = RotationQuaternion(eigenQuat1);
  const RotationQuaternion rotQuat2 = RotationQuaternion(eigenQuat2);
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(eigenQuatIdentity);

  const Vector vec = Vector(0.3,-1.5,0.6);
  const Vector vecX = Vector(1.0,0.0,0.0);
  const Vector vecY = Vector(0.0,1.0,0.0);
  const Vector vecZ = Vector(0.0,0.0,1.0);
  const Vector vecXSmallError = Vector(1.0000001,0.0,0.0);
  const RotationQuaternion rotQuatQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuatQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuatQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
};

template <typename RotationQuaternionImplementation>
struct RotationQuaternionSingleActiveTest : public RotationQuaternionSingleTest<RotationQuaternionImplementation>{

};

template <typename RotationQuaternionImplementation>
struct RotationQuaternionSinglePassiveTest : public RotationQuaternionSingleTest<RotationQuaternionImplementation>{
};

template <typename QuaternionImplementationQuaternionPair>
struct RotationQuaternionPairTest : public ::testing::Test{
  typedef typename QuaternionImplementationQuaternionPair::first_type RotationQuaternionFirstPrimType;
  typedef typename RotationQuaternionFirstPrimType::Scalar FirstScalar;
  typedef typename QuaternionImplementationQuaternionPair::second_type RotationQuaternionSecondPrimType;
  typedef typename RotationQuaternionSecondPrimType::Scalar SecondScalar;

  const RotationQuaternionFirstPrimType rotQuat1 = RotationQuaternionFirstPrimType(0.0,0.36,0.48,0.8);
  const RotationQuaternionSecondPrimType rotQuat2 = RotationQuaternionSecondPrimType(0.0,0.36,0.48,0.8);
  const typename quat::UnitQuaternion<FirstScalar> uquat1 = typename quat::UnitQuaternion<FirstScalar>(0.0,0.36,0.48,0.8);
  const typename quat::UnitQuaternion<SecondScalar> uquat2 = typename quat::UnitQuaternion<SecondScalar>(0.0,0.36,0.48,0.8);
  const typename quat::Quaternion<FirstScalar> quat1 = typename quat::Quaternion<FirstScalar>(0.0,0.36,0.48,0.8);
  const typename quat::Quaternion<SecondScalar> quat2 = typename quat::Quaternion<SecondScalar>(0.0,0.36,0.48,0.8);
};

typedef ::testing::Types<
    rot::RotationQuaternionPD,
    rot::RotationQuaternionPF
> RotationQuaternionTypes;

typedef ::testing::Types<
    rot::RotationQuaternionPD,
    rot::RotationQuaternionPF
> RotationQuaternionPassiveTypes;


typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPF, rot::RotationQuaternionPD>
> TypeQuaternionPairs;


TYPED_TEST_CASE(RotationQuaternionSingleTest, RotationQuaternionTypes);
TYPED_TEST_CASE(RotationQuaternionSinglePassiveTest, RotationQuaternionPassiveTypes);
TYPED_TEST_CASE(RotationQuaternionPairTest, TypeQuaternionPairs);



// --------------------------------------------------------------------------------------------------- //
// ------------------------------ Testing for Rotation Quaternions only ------------------------------ //
// --------------------------------------------------------------------------------------------------- //

// Test Rotation Quaternion Constructors and access operator (relies on casting to base)
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionConstructors){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rot;
  ASSERT_EQ(rot.w(), this->eigenQuatIdentity.w());
  ASSERT_EQ(rot.x(), this->eigenQuatIdentity.x());
  ASSERT_EQ(rot.y(), this->eigenQuatIdentity.y());
  ASSERT_EQ(rot.z(), this->eigenQuatIdentity.z());

  RotationQuaternion rot2(this->eigenQuat1.w(),this->eigenQuat1.x(),this->eigenQuat1.y(),this->eigenQuat1.z());
  ASSERT_NEAR(rot2.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->eigenQuat1.z(),1e-6);

  RotationQuaternion rot3(this->eigenQuat1);
  ASSERT_NEAR(rot3.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot3.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot3.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot3.z(), this->eigenQuat1.z(),1e-6);

  RotationQuaternion rot4(this->quat1);
  ASSERT_NEAR(rot4.w(), this->quat1.w(),1e-6);
  ASSERT_NEAR(rot4.x(), this->quat1.x(),1e-6);
  ASSERT_NEAR(rot4.y(), this->quat1.y(),1e-6);
  ASSERT_NEAR(rot4.z(), this->quat1.z(),1e-6);

  RotationQuaternion rot5(rot3);
  ASSERT_NEAR(rot5.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot5.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot5.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot5.z(), this->eigenQuat1.z(),1e-6);

  RotationQuaternion rot6(this->quat1.real(),this->quat1.imaginary());
  ASSERT_NEAR(rot6.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot6.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot6.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot6.z(), this->eigenQuat1.z(),1e-6);
  ASSERT_NEAR(rot6.real(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot6.imaginary()(0), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot6.imaginary()(1), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot6.imaginary()(2), this->eigenQuat1.z(),1e-6);
}

// Test Rotation Quaternion Assignment Operator (including test between different primtypes)
TYPED_TEST(RotationQuaternionPairTest, testRotationQuaternionAssignmentPrimTypes){
  typedef typename TestFixture::RotationQuaternionFirstPrimType RotationQuaternionFirstPrimType;
  typedef typename TestFixture::RotationQuaternionSecondPrimType RotationQuaternionSecondPrimType;
  RotationQuaternionFirstPrimType rot1;
  RotationQuaternionSecondPrimType rot2;

  //  RotationQuaternionXF = RotationQuaternionXF
  rot1 = this->rotQuat1;
  ASSERT_NEAR(rot1.w(), this->rotQuat1.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->rotQuat1.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->rotQuat1.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->rotQuat1.z(),1e-6);

  //  RotationQuaternionXF = RotationQuaternionXD
  rot1 = this->rotQuat2;
  ASSERT_NEAR(rot1.w(), this->rotQuat2.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->rotQuat2.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->rotQuat2.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->rotQuat2.z(),1e-6);

  //  RotationQuaternionXF = UnitQuaternionF
  rot1 = this->uquat1;
  ASSERT_NEAR(rot1.w(), this->uquat1.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->uquat1.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->uquat1.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->uquat1.z(),1e-6);

  //  RotationQuaternionXF = UnitQuaternionD
  rot1 = this->uquat2;
  ASSERT_NEAR(rot1.w(), this->uquat2.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->uquat2.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->uquat2.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->uquat2.z(),1e-6);

  //  RotationQuaternionXD = RotationQuaternionXD
  rot2 = this->rotQuat2;
  ASSERT_NEAR(rot2.w(), this->rotQuat2.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->rotQuat2.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->rotQuat2.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->rotQuat2.z(),1e-6);

  //  RotationQuaternionXD = RotationQuaternionXF
  rot2 = this->rotQuat1;
  ASSERT_NEAR(rot2.w(), this->rotQuat1.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->rotQuat1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->rotQuat1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->rotQuat1.z(),1e-6);

  //  RotationQuaternionXD = UnitQuaternionD
  rot2 = this->uquat2;
  ASSERT_NEAR(rot2.w(), this->uquat2.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->uquat2.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->uquat2.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->uquat2.z(),1e-6);

  //  RotationQuaternionXD = UnitQuaternionF
  rot2 = this->uquat1;
  ASSERT_NEAR(rot2.w(), this->uquat1.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->uquat1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->uquat1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->uquat1.z(),1e-6);
}

// Test Rotation Quaternion () Operator for casting from Quaternion/UnitQuaternion (including test between different primtypes)
TYPED_TEST(RotationQuaternionPairTest, testRotationQuaternionParenthesisPrimTypes){
  typedef typename TestFixture::RotationQuaternionFirstPrimType RotationQuaternionFirstPrimType;
  typedef typename TestFixture::RotationQuaternionSecondPrimType RotationQuaternionSecondPrimType;
  RotationQuaternionFirstPrimType rot1;
  RotationQuaternionSecondPrimType rot2;

  //  RotationQuaternionXF(RotationQuaternionXF)
  rot1(this->rotQuat1);
  ASSERT_NEAR(rot1.w(), this->rotQuat1.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->rotQuat1.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->rotQuat1.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->rotQuat1.z(),1e-6);

  //  RotationQuaternionXF(RotationQuaternionXD)
  rot1(this->rotQuat2);
  ASSERT_NEAR(rot1.w(), this->rotQuat2.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->rotQuat2.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->rotQuat2.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->rotQuat2.z(),1e-6);

  //  RotationQuaternionXD(RotationQuaternionXF)
  rot2(this->rotQuat1);
  ASSERT_NEAR(rot2.w(), this->rotQuat1.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->rotQuat1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->rotQuat1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->rotQuat1.z(),1e-6);

  //  RotationQuaternionXD(RotationQuaternionXD)
  rot2(this->rotQuat2);
  ASSERT_NEAR(rot2.w(), this->rotQuat2.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->rotQuat2.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->rotQuat2.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->rotQuat2.z(),1e-6);

  //  RotationQuaternionXF(UnitQuaternionF)
  rot1(this->uquat1);
  ASSERT_NEAR(rot1.w(), this->uquat1.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->uquat1.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->uquat1.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->uquat1.z(),1e-6);

  //  RotationQuaternionXF(UnitQuaternionD)
  rot1(this->uquat2);
  ASSERT_NEAR(rot1.w(), this->uquat2.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->uquat2.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->uquat2.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->uquat2.z(),1e-6);

  //  RotationQuaternionXD(UnitQuaternionD)
  rot2(this->uquat2);
  ASSERT_NEAR(rot2.w(), this->uquat2.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->uquat2.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->uquat2.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->uquat2.z(),1e-6);

  //  RotationQuaternionXD(UnitQuaternionF)
  rot2(this->uquat1);
  ASSERT_NEAR(rot2.w(), this->uquat1.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->uquat1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->uquat1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->uquat1.z(),1e-6);

  //  RotationQuaternionXF(QuaternionF)
  rot1(this->quat1);
  ASSERT_NEAR(rot1.w(), this->quat1.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->quat1.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->quat1.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->quat1.z(),1e-6);

  //  RotationQuaternionXF(QuaternionD)
  rot1(this->quat2);
  ASSERT_NEAR(rot1.w(), this->quat2.w(),1e-6);
  ASSERT_NEAR(rot1.x(), this->quat2.x(),1e-6);
  ASSERT_NEAR(rot1.y(), this->quat2.y(),1e-6);
  ASSERT_NEAR(rot1.z(), this->quat2.z(),1e-6);

  //  RotationQuaternionXD(QuaternionD)
  rot2(this->quat2);
  ASSERT_NEAR(rot2.w(), this->quat2.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->quat2.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->quat2.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->quat2.z(),1e-6);

  //  RotationQuaternionXD(QuaternionF)
  rot2(this->quat1);
  ASSERT_NEAR(rot2.w(), this->quat1.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->quat1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->quat1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->quat1.z(),1e-6);
}

// Test Rotation Quaternion Inversion
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rot = this->rotQuat1.inverted();
  ASSERT_NEAR(rot.w(), this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuat1Conj.z(),1e-6);

  RotationQuaternion rot2 = rot.invert();
  ASSERT_NEAR(rot.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuat1.z(),1e-6);
  ASSERT_NEAR(rot2.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->eigenQuat1.z(),1e-6);
}

// Test Rotation Quaternion Conjugation
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionConjugation){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rot = this->rotQuat1.conjugated();
  ASSERT_NEAR(rot.w(), this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuat1Conj.z(),1e-6);

  RotationQuaternion rot2 = rot.conjugate();
  ASSERT_NEAR(rot.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuat1.z(),1e-6);
  ASSERT_NEAR(rot2.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot2.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->eigenQuat1.z(),1e-6);
}

// Test Rotation Quaternion Norm
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionNorm){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rot = this->rotQuat1;
  ASSERT_NEAR(rot.norm(), 1.0,1e-6);
  rot = this->rotQuat2;
  ASSERT_NEAR(rot.norm(), 1.0,1e-6);
  rot = this->rotQuatIdentity;
  ASSERT_NEAR(rot.norm(), 1.0,1e-6);
}

// Test Rotation Quaternion Setters
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionSetters){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rot(this->rotQuat1);
  rot.setIdentity();
  ASSERT_NEAR(rot.w(), this->eigenQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuatIdentity.z(),1e-6);

  rot.setValues(this->eigenQuat1.w(), this->eigenQuat1.x(), this->eigenQuat1.y(), this->eigenQuat1.z());
  ASSERT_NEAR(rot.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuat1.z(),1e-6);

  rot.setIdentity();
  ASSERT_NEAR(rot.w(), this->eigenQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuatIdentity.z(),1e-6);

  rot.setParts(this->quat1.real(), this->quat1.imaginary());
  ASSERT_NEAR(rot.w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuat1.z(),1e-6);

  rot.setFromVectors(this->vec, this->vec);
  ASSERT_NEAR(rot.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotQuatIdentity.z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecY);
  ASSERT_NEAR(rot.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotQuatQuarterZ.z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecX);
  ASSERT_NEAR(rot.w(), this->eigenQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuatIdentity.z(),1e-6);


  rot.setFromVectors(this->vecX, this->vecXSmallError);
  ASSERT_NEAR(rot.w(), this->eigenQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rot.x(), this->eigenQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenQuatIdentity.z(),1e-6);

}

// Test Rotation Quaternion comparison (equality)
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionEqualityComparison){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  // Check equality comparison
  rotQuat = this->rotQuat1;
  ASSERT_EQ(rotQuat==this->rotQuat1,true);
  ASSERT_EQ(rotQuat==this->rotQuat2,false);
}

// Test Rotation Quaternion comparison (inequality)
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionInequalityComparison){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  // Check inequality comparison
  rotQuat = this->rotQuat1;
  ASSERT_EQ(rotQuat!=this->rotQuat1,false);
  ASSERT_EQ(rotQuat!=this->rotQuat2,true);
}


// Test Rotation Quaternion isNear
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionIsNear){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  // Check isNear
  rotQuat = this->rotQuat1;
  ASSERT_EQ(rotQuat.isNear(this->rotQuat1,1e-6),true);
  ASSERT_EQ(rotQuat.isNear(this->rotQuat2,1e-6),false);
}

// Test Rotation Quaternion Get/Set Unique
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionUniqueness){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;
  RotationQuaternion rotQuat2;

  // Check uniqueness getter and setter with generic Rotation Quaternions
  rotQuat = RotationQuaternion(-1.0,0.0,0.0,0.0);
  ASSERT_NEAR(rotQuat.getUnique().w(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), 0.0,1e-6);
  rotQuat = RotationQuaternion(0.0,-1.0,0.0,0.0);
  ASSERT_NEAR(rotQuat.getUnique().w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), 0.0,1e-6);
  rotQuat = RotationQuaternion(0.0,0.0,-1.0,0.0);
  ASSERT_NEAR(rotQuat.getUnique().w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), 0.0,1e-6);
  rotQuat = RotationQuaternion(0.0,0.0,0.0,-1.0);
  ASSERT_NEAR(rotQuat.getUnique().w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), 1.0,1e-6);
  rotQuat = RotationQuaternion(-1.0,0.0,0.0,0.0);
  rotQuat2 = rotQuat.setUnique();
  ASSERT_NEAR(rotQuat.w(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat.x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.z(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.w(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat2.x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.z(), 0.0,1e-6);
  rotQuat = RotationQuaternion(0.0,-1.0,0.0,0.0);
  rotQuat2 = rotQuat.setUnique();
  ASSERT_NEAR(rotQuat.w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.x(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat.y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.z(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.x(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat2.y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.z(), 0.0,1e-6);
  rotQuat = RotationQuaternion(0.0,0.0,-1.0,0.0);
  rotQuat2 = rotQuat.setUnique();
  ASSERT_NEAR(rotQuat.w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.y(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat.z(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.y(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat2.z(), 0.0,1e-6);
  rotQuat = RotationQuaternion(0.0,0.0,0.0,-1.0);
  rotQuat2 = rotQuat.setUnique();
  ASSERT_NEAR(rotQuat.w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat.z(), 1.0,1e-6);
  ASSERT_NEAR(rotQuat2.w(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.x(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.y(), 0.0,1e-6);
  ASSERT_NEAR(rotQuat2.z(), 1.0,1e-6);
}

// Test Rotation Quaternion Concatenation
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionConcatenation){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  // Check result of multiplication of a generic rotation quaternion with identity
  rotQuat = this->rotQuat1*this->rotQuatIdentity;
  ASSERT_EQ(rotQuat==this->rotQuat1,true);
  rotQuat = this->rotQuatIdentity*this->rotQuat1;
  ASSERT_EQ(rotQuat==this->rotQuat1,true);

  // Check concatenation of 4 quarters
  rotQuat = this->rotQuatQuarterX*this->rotQuatQuarterX*this->rotQuatQuarterX*this->rotQuatQuarterX;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatIdentity.getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatIdentity.getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatIdentity.getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatIdentity.getUnique().z(),1e-6);
  rotQuat = this->rotQuatQuarterY*this->rotQuatQuarterY*this->rotQuatQuarterY*this->rotQuatQuarterY;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatIdentity.getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatIdentity.getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatIdentity.getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatIdentity.getUnique().z(),1e-6);
  rotQuat = this->rotQuatQuarterZ*this->rotQuatQuarterZ*this->rotQuatQuarterZ*this->rotQuatQuarterZ;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatIdentity.getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatIdentity.getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatIdentity.getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatIdentity.getUnique().z(),1e-6);

  // Check concatenation of 3 different quarters
  rotQuat = this->rotQuatQuarterX.inverted()*this->rotQuatQuarterY*this->rotQuatQuarterX;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatQuarterZ.inverted().getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatQuarterZ.inverted().getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatQuarterZ.inverted().getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatQuarterZ.inverted().getUnique().z(),1e-6);
  rotQuat = this->rotQuatQuarterX.inverted()*this->rotQuatQuarterZ*this->rotQuatQuarterX;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatQuarterY.getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatQuarterY.getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatQuarterY.getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatQuarterY.getUnique().z(),1e-6);
  rotQuat = this->rotQuatQuarterY.inverted()*this->rotQuatQuarterX*this->rotQuatQuarterY;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatQuarterZ.getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatQuarterZ.getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatQuarterZ.getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatQuarterZ.getUnique().z(),1e-6);
  rotQuat = this->rotQuatQuarterY.inverted()*this->rotQuatQuarterZ*this->rotQuatQuarterY;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatQuarterX.inverted().getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatQuarterX.inverted().getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatQuarterX.inverted().getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatQuarterX.inverted().getUnique().z(),1e-6);
  rotQuat = this->rotQuatQuarterZ.inverted()*this->rotQuatQuarterX*this->rotQuatQuarterZ;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatQuarterY.inverted().getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatQuarterY.inverted().getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatQuarterY.inverted().getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatQuarterY.inverted().getUnique().z(),1e-6);
  rotQuat = this->rotQuatQuarterZ.inverted()*this->rotQuatQuarterY*this->rotQuatQuarterZ;
  ASSERT_NEAR(rotQuat.getUnique().w(), this->rotQuatQuarterX.getUnique().w(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().x(), this->rotQuatQuarterX.getUnique().x(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().y(), this->rotQuatQuarterX.getUnique().y(),1e-6);
  ASSERT_NEAR(rotQuat.getUnique().z(), this->rotQuatQuarterX.getUnique().z(),1e-6);
}

// Testing of special matrices
TYPED_TEST (RotationQuaternionSingleTest, testRotationQuaternionSingleSpecialMatrices) {
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  RotationQuaternion quat1 = this->rotQuat1;
  RotationQuaternion quat2 = this->rotQuat2;

  // Qleft
  RotationQuaternion concatenation1 = quat1 * quat2;
  RotationQuaternion concatenation2(quat1.getQuaternionMatrix() * quat2.vector());
  ASSERT_NEAR(concatenation1.w(),concatenation2.w(), 1e-3);
  ASSERT_NEAR(concatenation1.x(),concatenation2.x(), 1e-3);
  ASSERT_NEAR(concatenation1.y(),concatenation2.y(), 1e-3);
  ASSERT_NEAR(concatenation1.z(),concatenation2.z(), 1e-3);

  // Qright
  RotationQuaternion concatenation3 = quat1 * quat2;
  RotationQuaternion concatenation4(quat2.getConjugateQuaternionMatrix() * quat1.vector());
  ASSERT_NEAR(concatenation3.w(),concatenation4.w(), 1e-3);
  ASSERT_NEAR(concatenation3.x(),concatenation4.x(), 1e-3);
  ASSERT_NEAR(concatenation3.y(),concatenation4.y(), 1e-3);
  ASSERT_NEAR(concatenation3.z(),concatenation4.z(), 1e-3);
}


// Test fix
TYPED_TEST(RotationQuaternionSingleTest, testFix){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rot;
  RotationQuaternion rotModified;

  // Check fix
  rot = this->rotQuat1;
  rotModified.toImplementation().w() = rot.toImplementation().w()*1.1;
  rotModified.toImplementation().x() = rot.toImplementation().x()*1.1;
  rotModified.toImplementation().y() = rot.toImplementation().y()*1.1;
  rotModified.toImplementation().z() = rot.toImplementation().z()*1.1;
  rotModified.fix();
  ASSERT_NEAR(rotModified.w(), rot.w(),1e-6);
  ASSERT_NEAR(rotModified.x(), rot.x(),1e-6);
  ASSERT_NEAR(rotModified.y(), rot.y(),1e-6);
  ASSERT_NEAR(rotModified.z(), rot.z(),1e-6);
}

// Test Rotation Quaternion Vector Rotation
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionVectorRotation){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  RotationQuaternion rotQuat;
  Vector testVec;
  Vector testVec1;
  Vector testVec2;

  int signSwitch = 1;

  // Check rotation of base vectors around main axis
  testVec = this->rotQuatQuarterX.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotQuatQuarterX.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotQuatQuarterX.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),1e-6);
  testVec = this->rotQuatQuarterY.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotQuatQuarterY.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotQuatQuarterY.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),1e-6);
  testVec = this->rotQuatQuarterZ.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),1e-6);
  testVec = this->rotQuatQuarterZ.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),1e-6);
  testVec = this->rotQuatQuarterZ.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  testVec = this->rotQuatQuarterX.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotQuatQuarterX.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotQuatQuarterX.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),1e-6);
  testVec = this->rotQuatQuarterY.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotQuatQuarterY.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotQuatQuarterY.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),1e-6);
  testVec = this->rotQuatQuarterZ.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),1e-6);
  testVec = this->rotQuatQuarterZ.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),1e-6);
  testVec = this->rotQuatQuarterZ.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  // Check rotation with Identity
  testVec = this->rotQuatIdentity.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotQuatIdentity.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotQuatIdentity.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);
  testVec = this->rotQuatIdentity.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotQuatIdentity.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotQuatIdentity.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  // Check combination between concatenation and rotate
  testVec1 = this->rotQuat2.rotate(this->rotQuat1.rotate(this->vec));
  testVec2 = (this->rotQuat2*this->rotQuat1).rotate(this->vec);
  ASSERT_NEAR(testVec1(0), testVec2(0),1e-6);
  ASSERT_NEAR(testVec1(1), testVec2(1),1e-6);
  ASSERT_NEAR(testVec1(2), testVec2(2),1e-6);
}

// Test Rotation Quaternion cast to UnitQuaternion
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionUnitQuaternionCast){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  UnitQuaternion unitQuat = this->rotQuat1.toUnitQuaternion();
  ASSERT_NEAR(unitQuat.w(), this->rotQuat1.w(),1e-6);
  ASSERT_NEAR(unitQuat.x(), this->rotQuat1.x(),1e-6);
  ASSERT_NEAR(unitQuat.y(), this->rotQuat1.y(),1e-6);
  ASSERT_NEAR(unitQuat.z(), this->rotQuat1.z(),1e-6);
}

// Test Rotation Quaternion cast to Implementation
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionToImplementationCast){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::EigenQuat EigenQuat;
  typedef typename TestFixture::Scalar Scalar;

  EigenQuat eigenQuat = this->rotQuat1.toImplementation();
  ASSERT_NEAR(eigenQuat.w(), this->rotQuat1.w(),1e-6);
  ASSERT_NEAR(eigenQuat.x(), this->rotQuat1.x(),1e-6);
  ASSERT_NEAR(eigenQuat.y(), this->rotQuat1.y(),1e-6);
  ASSERT_NEAR(eigenQuat.z(), this->rotQuat1.z(),1e-6);
}

//// Test Rotation Quaternion cast to StoredUnitQuaternion versus cast to StoredImplementation
//TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionStoredUnitImplementationCast){
//  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
//  typedef typename TestFixture::UnitQuaternion UnitQuaternion;
//  typedef typename TestFixture::EigenQuat EigenQuat;
//  typedef typename TestFixture::Scalar Scalar;
//
//  EigenQuat eigenQuat = this->rotQuat1.toImplementation();
//  UnitQuaternion unitQuat = this->rotQuat1.toUnitQuaternion();
//  ASSERT_NEAR(eigenQuat.w(), unitQuat.w(),1e-6);
//  ASSERT_NEAR(eigenQuat.x(), unitQuat.x(),1e-6);
//  ASSERT_NEAR(eigenQuat.y(), unitQuat.y(),1e-6);
//  ASSERT_NEAR(eigenQuat.z(), unitQuat.z(),1e-6);
//}

// Test Rotation Quaternion getDisparityAngle
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionGetDisparityAngle){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  ASSERT_NEAR(this->rotQuat1.getDisparityAngle(this->rotQuat1),0.0,1e-6);
  ASSERT_NEAR(this->rotQuat2.getDisparityAngle(this->rotQuat2),0.0,1e-6);
  ASSERT_NEAR(this->rotQuatIdentity.getDisparityAngle(this->rotQuatIdentity),0.0,1e-6);
  ASSERT_NEAR(this->rotQuat2.getDisparityAngle(this->rotQuat1),this->rotQuat1.getDisparityAngle(this->rotQuat2),1e-6);
  ASSERT_NEAR(this->rotQuat1.getDisparityAngle(this->rotQuatIdentity),calcRotationQuatDisparityAngleToIdentity(this->rotQuat1),1e-6);
  ASSERT_NEAR(this->rotQuat2.getDisparityAngle(this->rotQuat1),calcRotationQuatDisparityAngle(this->rotQuat1, this->rotQuat2),1e-6);
}

// Test Rotation Quaternion Exponential and Logarithmic Map
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionExponentialMap){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;

  testVec = this->rotQuatIdentity.logarithmicMap();
  ASSERT_NEAR(testVec(0), 0.0,1e-6);
  ASSERT_NEAR(testVec(1), 0.0,1e-6);
  ASSERT_NEAR(testVec(2), 0.0,1e-6);

  testVec = this->rotQuat1.logarithmicMap();
  RotationQuaternion rotQuatToExpMap = RotationQuaternion::exponentialMap(testVec);
  ASSERT_EQ(rotQuatToExpMap.isNear(this->rotQuat1,1e-6),true);

  testVec = this->rotQuat2.logarithmicMap();
  rotQuatToExpMap = RotationQuaternion::exponentialMap(testVec);
  ASSERT_EQ(rotQuatToExpMap.isNear(this->rotQuat2,1e-6),true);

  double norm = 0.1;
  // --------------------------------------------------------------------------------------------------- //
  // -------- Testing for casting between different type of rotations and rotation Quaternions --------- //
  // --------------------------------------------------------------------------------------------------- //
  testVec = this->vec/this->vec.norm()*norm;
  rotQuatToExpMap = RotationQuaternion::exponentialMap(testVec);
  ASSERT_NEAR(rotQuatToExpMap.getDisparityAngle(this->rotQuatIdentity),norm,1e-6);

  testVec.setZero();
  rotQuatToExpMap = RotationQuaternion::exponentialMap(testVec);
  ASSERT_NEAR(rotQuatToExpMap.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuatToExpMap.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuatToExpMap.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuatToExpMap.z(), this->rotQuatIdentity.z(),1e-6);
}

// Test Rotation Quaternion boxplus and boxminus
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionBoxOperators){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  RotationQuaternion rotQuat;
  RotationQuaternion rotQuat2;
  Vector testVec;

  // Test addition with 0
  testVec.setZero();
  rotQuat = this->rotQuat1.boxPlus(testVec);
  ASSERT_EQ(rotQuat.isNear(this->rotQuat1,1e-6),true);

  // Test substraction of same elements
  testVec = this->rotQuat1.boxMinus(this->rotQuat1);
  ASSERT_NEAR(testVec(0),0.0,1e-6);
  ASSERT_NEAR(testVec(1),0.0,1e-6);
  ASSERT_NEAR(testVec(2),0.0,1e-6);

  // Test backward-forward
  testVec = this->rotQuat1.boxMinus(this->rotQuat2);
  rotQuat = this->rotQuat2.boxPlus(testVec);
  ASSERT_EQ(rotQuat.isNear(this->rotQuat1,1e-6),true);

  // Test forward-backward
  testVec = this->vec;
  rotQuat = this->rotQuat1.boxPlus(testVec);
  testVec = rotQuat.boxMinus(this->rotQuat1);
  ASSERT_NEAR(testVec(0),this->vec(0),1e-6);
  ASSERT_NEAR(testVec(1),this->vec(1),1e-6);
  ASSERT_NEAR(testVec(2),this->vec(2),1e-6);

  // Test special case
  testVec = Vector(5.45377e-10,9.59447e-10,-2.18031e-12);
  rotQuat.setValues(1,-1.76722e-15,2.01142e-15,-4.71845e-15);
  rotQuat2 = rotQuat.boxPlus(testVec);
  ASSERT_EQ(std::isnan(rotQuat2.w()),false) << rotQuat2;


  // Test forward-backward with small angle
  testVec = this->vec*1e-12;
  rotQuat = this->rotQuat1.boxPlus(testVec);
  ASSERT_EQ(std::isnan(rotQuat.w()),false);
  testVec = rotQuat.boxMinus(this->rotQuat1);
  ASSERT_NEAR(testVec(0),this->vec(0)*1e-12,1e-6);
  ASSERT_NEAR(testVec(1),this->vec(1)*1e-12,1e-6);
  ASSERT_NEAR(testVec(2),this->vec(2)*1e-12,1e-6);

  // Test overlap with disparity angle
  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rotQuat = this->rotQuat1.boxPlus(testVec);
  ASSERT_NEAR(rotQuat.getDisparityAngle(this->rotQuat1),norm,1e-4); // Check distance between both Rotation Quaternions
  rotQuat2 = this->rotQuat1.boxPlus(2*testVec);
  ASSERT_NEAR(rotQuat.getDisparityAngle(rotQuat2),norm,1e-4); // Check distance to double
  rotQuat2 = this->rotQuat1.boxPlus(-testVec);
  ASSERT_NEAR(rotQuat.getDisparityAngle(rotQuat2),2*norm,1e-4); // Check distance to reverse

  // box minus (bug)
  Scalar h = 1.0e-8;
  RotationQuaternion rotA(-0.96172135575480655, -0.071670794727345111, -0.25459502556731622, -0.071670803153484147);
  RotationQuaternion rotB( 0.96172135489322863,  0.07167080867178631,   0.25459502571498405,  0.071670800245647273);
  std::cout << "------------------------------> compute angularVel: \n";
  Vector angularVel = rotB.boxMinus(rotA)/(2.0*h);
  std::cout << "------------------------------> compute angularVel2: \n";
  Vector angularVel2 = rotB.boxMinus(rotA.getUnique())/(2.0*h);
  EXPECT_EQ(angularVel.x(), angularVel2.x());
  EXPECT_EQ(angularVel.y(), angularVel2.y());
  EXPECT_EQ(angularVel.z(), angularVel2.z());
  std::cout << "angularVel: " << angularVel.transpose() << std::endl;
  std::cout << "angularVel2: " << angularVel2.transpose() << std::endl;

}


TYPED_TEST(RotationQuaternionSingleTest, testSetRandom){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  RotationQuaternion rotA;
  rotA.setRandom();
  ASSERT_TRUE(!rotA.isNear(RotationQuaternion(), 1e-6)) << "rotA: " << rotA;

  // check if setRandom returns by reference
  rotA.setRandom().setIdentity();
  ASSERT_TRUE(rotA.isNear(RotationQuaternion(), 1e-6));
}

TYPED_TEST(RotationQuaternionSingleTest, testGetRandom){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  RotationQuaternion rotA;
  rotA.setRandom();
  RotationQuaternion rotB = rotA;
  RotationQuaternion rotC;
  rotC = RotationQuaternion::getRandom();

  ASSERT_TRUE(rotB.isNear(rotA, 1e-4));
  ASSERT_TRUE(!rotC.isNear(rotA, 1e-4));
}
