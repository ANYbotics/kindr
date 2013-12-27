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

#include "kindr/common/gtest_eigen.hpp"
#include "kindr/quaternions/QuaternionEigen.hpp"
#include "kindr/rotations/RotationEigen.hpp"

namespace rot = kindr::rotations::eigen_impl;
namespace quat = kindr::quaternions::eigen_impl;



template <typename RotationVectorImplementation>
class RotationVectorSingleTest : public ::testing::Test{
 public:
  typedef RotationVectorImplementation RotationVector;
  typedef typename RotationVectorImplementation::Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,1> Vector;

  const Vector vec1 = Vector(0.36,0.48,0.8);
  const Vector vec2 = Vector(0.3,2.0,0.0);
  const Vector vecZero = Vector(0.0,0.0,0.0);

  const RotationVector rotVec1 = RotationVector(vec1);
  const RotationVector rotVec2 = RotationVector(vec2);
  const RotationVector rotVecIdentity = RotationVector(vecZero);
};

template <typename RotationQuaternionRotationVectorImplementationPair>
struct RotationQuaternionRotationVectorPairTest : public ::testing::Test{
  typedef typename RotationQuaternionRotationVectorImplementationPair::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationQuaternionRotationVectorImplementationPair::second_type RotationVector;
  typedef typename RotationVector::Scalar RotationVectorScalar;

  const RotationQuaternion rotQuatQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuatQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuatQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(1.0,0.0,0.0,0.0);
  const RotationQuaternion rotQuat1 = RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
  const RotationQuaternion rotQuat1Conj = RotationQuaternion(4.0/sqrt(30.0),-3.0/sqrt(30.0),-1.0/sqrt(30.0),-2.0/sqrt(30.0));

  const RotationVector rotVecQuarterX = RotationVector(M_PI/2.0,0.0,0.0);
  const RotationVector rotVecQuarterY = RotationVector(0.0,M_PI/2.0,0.0);
  const RotationVector rotVecQuarterZ = RotationVector(0.0,0.0,M_PI/2.0);
  const RotationVector rotVecIdentity = RotationVector(0.0,0.0,0.0);
};


typedef ::testing::Types<
    rot::RotationVectorPD,
    rot::RotationVectorPF,
    rot::RotationVectorAD,
    rot::RotationVectorAF
> RotationVectorTypes;


typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPF, rot::RotationVectorPF>,
    std::pair<rot::RotationQuaternionPF, rot::RotationVectorPD>,
    std::pair<rot::RotationQuaternionPD, rot::RotationVectorPF>,
    std::pair<rot::RotationQuaternionPD, rot::RotationVectorPD>,
    std::pair<rot::RotationQuaternionAF, rot::RotationVectorAF>,
    std::pair<rot::RotationQuaternionAF, rot::RotationVectorAD>,
    std::pair<rot::RotationQuaternionAD, rot::RotationVectorAF>,
    std::pair<rot::RotationQuaternionAD, rot::RotationVectorAD>
> TypeQuaternionRotationVectorPairs;


TYPED_TEST_CASE(RotationVectorSingleTest, RotationVectorTypes);
TYPED_TEST_CASE(RotationQuaternionRotationVectorPairTest, TypeQuaternionRotationVectorPairs);


// --------------------------------------------------------------------------------------------------- //
// ------------------ Testing for constructors and getters for other rotation types ------------------ //
// --------------------------------------------------------------------------------------------------- //

// Testing constructors and getters for Rotation Vector
TYPED_TEST(RotationVectorSingleTest, testRotationVectorConstructors){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;

  RotationVector rot;
  ASSERT_EQ(rot.x(), 0.0);
  ASSERT_EQ(rot.y(), 0.0);
  ASSERT_EQ(rot.z(), 0.0);

  RotationVector rot2(this->vec1.x(),this->vec1.y(),this->vec1.z());
  ASSERT_NEAR(rot2.x(), this->vec1.x(),1e-6);
  ASSERT_NEAR(rot2.y(), this->vec1.y(),1e-6);
  ASSERT_NEAR(rot2.z(), this->vec1.z(),1e-6);

  RotationVector rot3(this->vec1);
  ASSERT_NEAR(rot3.x(), this->vec1.x(),1e-6);
  ASSERT_NEAR(rot3.y(), this->vec1.y(),1e-6);
  ASSERT_NEAR(rot3.z(), this->vec1.z(),1e-6);

  RotationVector rot4(this->rotVec1);
  ASSERT_NEAR(rot4.x(), this->rotVec1.x(),1e-6);
  ASSERT_NEAR(rot4.y(), this->rotVec1.y(),1e-6);
  ASSERT_NEAR(rot4.z(), this->rotVec1.z(),1e-6);

  RotationVector rot5(rot4);
  ASSERT_NEAR(rot5.x(), this->rotVec1.x(),1e-6);
  ASSERT_NEAR(rot5.y(), this->rotVec1.y(),1e-6);
  ASSERT_NEAR(rot5.z(), this->rotVec1.z(),1e-6);
}

// TODO: do the same for all other types of rotation

// --------------------------------------------------------------------------------------------------- //
// -------- Testing for casting between different type of rotations and rotation Quaternions --------- //
// --------------------------------------------------------------------------------------------------- //

// Test convertion between rotation quaternion and rotation vectors
TYPED_TEST(RotationQuaternionRotationVectorPairTest, testConversionRotationQuaternionRotationVector){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationVector RotationVector;
  RotationQuaternion rotQuat;
  RotationVector rotVec;

  // TODO: add generic

  rotQuat = this->rotVecIdentity;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-6);
  rotQuat = this->rotVecQuarterX;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-6);
  rotQuat = this->rotVecQuarterY;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-6);
  rotQuat = this->rotVecQuarterZ;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-6);

  rotVec = this->rotQuatIdentity;
  ASSERT_NEAR(rotVec.x(), this->rotVecIdentity.x(),1e-6);
  ASSERT_NEAR(rotVec.y(), this->rotVecIdentity.y(),1e-6);
  ASSERT_NEAR(rotVec.z(), this->rotVecIdentity.z(),1e-6);
  rotVec = this->rotQuatQuarterX;
  ASSERT_NEAR(rotVec.x(), this->rotVecQuarterX.x(),1e-6);
  ASSERT_NEAR(rotVec.y(), this->rotVecQuarterX.y(),1e-6);
  ASSERT_NEAR(rotVec.z(), this->rotVecQuarterX.z(),1e-6);
  rotVec = this->rotQuatQuarterY;
  ASSERT_NEAR(rotVec.x(), this->rotVecQuarterY.x(),1e-6);
  ASSERT_NEAR(rotVec.y(), this->rotVecQuarterY.y(),1e-6);
  ASSERT_NEAR(rotVec.z(), this->rotVecQuarterY.z(),1e-6);
  rotVec = this->rotQuatQuarterZ;
  ASSERT_NEAR(rotVec.x(), this->rotVecQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotVec.y(), this->rotVecQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotVec.z(), this->rotVecQuarterZ.z(),1e-6);
}

// --------------------------------------------------------------------------------------------------- //
// ------------------------------------- Testing Rotation Vector ------------------------------------- //
// --------------------------------------------------------------------------------------------------- //

// Test Rotation Vector Inversion
TYPED_TEST(RotationQuaternionRotationVectorPairTest, testRotationVectorInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationQuaternion RotationVector;
  RotationQuaternion rotQuat;
  RotationVector rotVec1;
  RotationVector rotVec2;

  // Use rotation quaternion method as reference
  rotQuat = this->rotQuat1.inverted();
  rotVec1 = rotQuat;

  // Use rotation vector method and compare
  rotVec2 = this->rotQuat1;
  rotVec2.invert();
  ASSERT_NEAR(rotVec1.x(),rotVec2.x(),1e-6);
  ASSERT_NEAR(rotVec1.y(),rotVec2.y(),1e-6);
  ASSERT_NEAR(rotVec1.z(),rotVec2.z(),1e-6);
}

// TODO: do the same for all other types of rotation and all other methods







