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


template <typename Rotation_>
class RotationVectorSingleTest : public ::testing::Test{
 public:
  typedef Rotation_ RotationVector;
  typedef typename Rotation_::Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,1> Vector;
  typedef typename rot::RotationQuaternion<Scalar, Rotation_::Usage> RotationQuaternion;
  const Vector eigenVector3Identity = Vector(0.0,0.0,0.0);
  const Vector eigenVector3v1 = Vector(0.36,0.48,0.8);
  const Vector eigenVector3v2 = Vector(0.3,2.0,0.0);

  const RotationVector rotRotationVectorV1 = RotationVector(eigenVector3v1);
  const RotationVector rotRotationVectorV2 = RotationVector(eigenVector3v2);

  const RotationVector rotRotationVector1 = RotationVector(RotationQuaternion(0.0,0.36,0.48,0.8));
  const RotationVector rotRotationVector2 = RotationVector(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)));

  const RotationVector rotRotationVectorQuarterX = RotationVector(M_PI/2.0,0.0,0.0);
  const RotationVector rotRotationVectorQuarterY = RotationVector(0.0,M_PI/2.0,0.0);
  const RotationVector rotRotationVectorQuarterZ = RotationVector(0.0,0.0,M_PI/2.0);
  const RotationVector rotRotationVectorIdentity = RotationVector(0.0,0.0,0.0);

  const Vector vec = Vector(1.3,-2.5,3.6);
  const Vector vecX = Vector(1.0,0.0,0.0);
  const Vector vecY = Vector(0.0,1.0,0.0);
  const Vector vecZ = Vector(0.0,0.0,1.0);
};

template <typename RotationQuaternionRotationVectorImplementationPair>
struct RotationQuaternionRotationVectorPairTest : public ::testing::Test{
  typedef typename RotationQuaternionRotationVectorImplementationPair::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationQuaternionRotationVectorImplementationPair::second_type RotationVector;
  typedef typename RotationVector::Scalar RotationVectorScalar;

  const RotationQuaternionScalar rotQuatSmallNumber = kindr::common::NumTraits<RotationQuaternionScalar>::dummy_precision()/10.0;
  const RotationQuaternionScalar rotVecSmallNumber = kindr::common::NumTraits<RotationVectorScalar>::dummy_precision()/10.0;


  const RotationQuaternion rotQuatQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuatQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuatQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(1.0,0.0,0.0,0.0);
  const RotationQuaternion rotQuat1 = RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
  const RotationQuaternion rotQuat1Conj = RotationQuaternion(4.0/sqrt(30.0),-3.0/sqrt(30.0),-1.0/sqrt(30.0),-2.0/sqrt(30.0));
//  const RotationQuaternion rotQuat2 = RotationQuaternion(rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber));

  const RotationVector rotRotationVectorQuarterX = RotationVector(M_PI/2.0,0.0,0.0);
  const RotationVector rotRotationVectorQuarterY = RotationVector(0.0,M_PI/2.0,0.0);
  const RotationVector rotRotationVectorQuarterZ = RotationVector(0.0,0.0,M_PI/2.0);
  const RotationVector rotRotationVectorIdentity = RotationVector(0.0,0.0,0.0);
//  const RotationVector rotRotationVectorV2 = RotationVector(2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber));
//  const RotationVector rotVec3 = RotationVector(rotVecSmallNumber,rotVecSmallNumber,rotVecSmallNumber);
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
TYPED_TEST(RotationVectorSingleTest, testConstructors){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;

  RotationVector rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3Identity, rot.toImplementation(), 1e-4, "constructor");

  RotationVector rot2(this->eigenVector3v1.x(),this->eigenVector3v1.y(),this->eigenVector3v1.z());
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot2.toImplementation(), 1e-4, "constructor");

  RotationVector rot3(this->eigenVector3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot3.toImplementation(), 1e-4, "constructor");

  RotationVector rot4(this->rotRotationVectorV1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot4.toImplementation(), 1e-4, "constructor");

  RotationVector rot5(rot4);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot5.toImplementation(), 1e-4, "constructor");
}

TYPED_TEST(RotationVectorSingleTest, testGetters)
{
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;

  RotationVector rot(this->eigenVector3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot.vector(), 1e-4, "vector()");
  ASSERT_NEAR(rot.x(), this->eigenVector3v1.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenVector3v1.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenVector3v1.z(),1e-6);

}

TYPED_TEST(RotationVectorSingleTest, testAssignmentOperator){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;

  RotationVector rot(this->eigenVector3v1);
  RotationVector rot1 = rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot1.toImplementation(), 1e-4, "constructor");

}

TYPED_TEST(RotationVectorSingleTest, testParenthesisOperator) {
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;

  RotationVector rot(this->eigenVector3v1);
  RotationVector rot1;
  rot1(rot);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot1.toImplementation(), 1e-4, "constructor");

}





TYPED_TEST(RotationVectorSingleTest, testSetters)
{
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;

  RotationVector rot(this->eigenVector3v1);
  rot.setIdentity();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3Identity, rot.toImplementation(), 1e-4, "identity");


}

/* Test Uniqueness
 *  Assumes conversion between rotation quaternion and rotation vector is correct.
 */
TYPED_TEST(RotationVectorSingleTest, testUniqueness){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationVector rotVec;
  RotationVector rotRotationVectorV2;
  RotationVector rotVecUnique;

  // Check uniqueness getter and setter with generic Rotation Quaternions
  rotVec = RotationQuaternion(-1.0,0.0,0.0,0.0);
  rotVecUnique =  RotationQuaternion(1.0,0.0,0.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotVec.getUnique().toImplementation(), 1e-4, "unique");

  rotVec = RotationQuaternion(0.0,-1.0,0.0,0.0);
  rotVecUnique = RotationQuaternion(0.0,1.0,0.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotVec.getUnique().toImplementation(), 1e-4, "unique");

  rotVec = RotationQuaternion(0.0,0.0,-1.0,0.0);
  rotVecUnique = RotationQuaternion(0.0,0.0,1.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotVec.getUnique().toImplementation(), 1e-4, "unique");

  rotVec = RotationQuaternion(0.0,0.0,0.0,-1.0);
  rotVecUnique = RotationQuaternion(0.0,0.0,0.0,1.0);
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotVec.getUnique().toImplementation(), 1e-4, "unique");

  rotVec = RotationQuaternion(-1.0,0.0,0.0,0.0);
  rotVecUnique = RotationQuaternion(1.0,0.0,0.0,0.0);
  rotRotationVectorV2 = rotVec.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotVec.toImplementation(), 1e-4, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotRotationVectorV2.toImplementation(), 1e-4, "unique");

  rotVec = RotationQuaternion(0.0,-1.0,0.0,0.0);
  rotVecUnique = RotationQuaternion(0.0,1.0,0.0,0.0);
  rotRotationVectorV2 = rotVec.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotVec.toImplementation(), 1e-4, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotRotationVectorV2.toImplementation(), 1e-4, "unique");

  rotVec = RotationQuaternion(0.0,0.0,-1.0,0.0);
  rotVecUnique = RotationQuaternion(0.0,0.0,1.0,0.0);
  rotRotationVectorV2 = rotVec.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotVec.toImplementation(), 1e-4, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotRotationVectorV2.toImplementation(), 1e-4, "unique");


  rotVec = RotationQuaternion(0.0,0.0,0.0,-1.0);
  rotVecUnique = RotationQuaternion(0.0,0.0,0.0,1.0);
  rotRotationVectorV2 = rotVec.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotVec.toImplementation(), 1e-4, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ(rotVecUnique.toImplementation(), rotRotationVectorV2.toImplementation(), 1e-4, "unique");

}

/* Test comparison (equality)
 *
 */
TYPED_TEST(RotationVectorSingleTest, testComparisonEqual){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;
  RotationVector rot;

  // Check equality comparison
  rot = this->rotRotationVectorV1;
  ASSERT_EQ(true, rot==this->rotRotationVectorV1);
  ASSERT_EQ(false, rot==this->rotRotationVectorV2);
}

/* Test  getDisparityAngle
 * Assumes conversion between RotationVector and RotationQuaternion is correct.
 */
TYPED_TEST(RotationVectorSingleTest, testGetDisparityAngle){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  ASSERT_NEAR(this->rotRotationVector1.getDisparityAngle(this->rotRotationVector1),0.0,1e-6);
  ASSERT_NEAR(this->rotRotationVector2.getDisparityAngle(this->rotRotationVector2),0.0,1e-6);
  ASSERT_NEAR(this->rotRotationVectorIdentity.getDisparityAngle(this->rotRotationVectorIdentity),0.0,1e-6);
  ASSERT_NEAR(this->rotRotationVector2.getDisparityAngle(this->rotRotationVector1),this->rotRotationVector1.getDisparityAngle(this->rotRotationVector2),1e-6);
  ASSERT_NEAR(this->rotRotationVector1.getDisparityAngle(this->rotRotationVectorIdentity),fabs(acos(RotationQuaternion(this->rotRotationVector1).w())*2),1e-6);
  ASSERT_NEAR(this->rotRotationVector2.getDisparityAngle(this->rotRotationVector1),fabs(acos((RotationQuaternion(this->rotRotationVector1).inverted()*RotationQuaternion(this->rotRotationVector2)).w())*2),1e-6);
}

/* Test isNear
 * Assumes that getDisparityAngle() is correct.
 */
TYPED_TEST(RotationVectorSingleTest, testIsNear){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;
  RotationVector rot;

  // Check isNear
  rot = this->rotRotationVectorV1;
  ASSERT_EQ(rot.isNear(this->rotRotationVectorV1,1e-6),true);
  ASSERT_EQ(rot.isNear(this->rotRotationVectorV2,1e-6),false);
}



/* Test Concatenation
 * Assumes isNear is correct.
 */
TYPED_TEST(RotationVectorSingleTest, testConcatenation){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;
  RotationVector rotRotationVector;

  // Check result of multiplication of a generic rotation with identity
  rotRotationVector = this->rotRotationVector1*this->rotRotationVectorIdentity;
  ASSERT_EQ(rotRotationVector.isNear(this->rotRotationVector1,1e-6),true);
  rotRotationVector = this->rotRotationVectorIdentity*this->rotRotationVector1;
  ASSERT_EQ(rotRotationVector.isNear(this->rotRotationVector1,1e-6),true);

  // Check concatenation of 4 quarters
  rotRotationVector = this->rotRotationVectorQuarterX*this->rotRotationVectorQuarterX*this->rotRotationVectorQuarterX*this->rotRotationVectorQuarterX;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorIdentity.toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");

  rotRotationVector = this->rotRotationVectorQuarterY*this->rotRotationVectorQuarterY*this->rotRotationVectorQuarterY*this->rotRotationVectorQuarterY;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorIdentity.toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");


  rotRotationVector = this->rotRotationVectorQuarterZ*this->rotRotationVectorQuarterZ*this->rotRotationVectorQuarterZ*this->rotRotationVectorQuarterZ;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorIdentity.toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");

  // Check concatenation of 3 different quarters
  rotRotationVector = this->rotRotationVectorQuarterX.inverted()*this->rotRotationVectorQuarterY*this->rotRotationVectorQuarterX;
  if(RotationVector::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationVector.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorQuarterZ.toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");

  rotRotationVector = this->rotRotationVectorQuarterX.inverted()*this->rotRotationVectorQuarterZ*this->rotRotationVectorQuarterX;
  if(RotationVector::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationVector.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorQuarterY.inverted().toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");

   rotRotationVector = this->rotRotationVectorQuarterY.inverted()*this->rotRotationVectorQuarterX*this->rotRotationVectorQuarterY;
  if(RotationVector::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationVector.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorQuarterZ.inverted().toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");

  rotRotationVector = this->rotRotationVectorQuarterY.inverted()*this->rotRotationVectorQuarterZ*this->rotRotationVectorQuarterY;
  if(RotationVector::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationVector.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorQuarterX.toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");

  rotRotationVector = this->rotRotationVectorQuarterZ.inverted()*this->rotRotationVectorQuarterX*this->rotRotationVectorQuarterZ;
  if(RotationVector::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationVector.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorQuarterY.toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");

  rotRotationVector = this->rotRotationVectorQuarterZ.inverted()*this->rotRotationVectorQuarterY*this->rotRotationVectorQuarterZ;
  if(RotationVector::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationVector.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorQuarterX.inverted().toImplementation(), rotRotationVector.toImplementation(), 1e-4, "concatenation");

}


// Test Vector Rotation
TYPED_TEST(RotationVectorSingleTest, testVectorRotation){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;
  Vector testVec1;
  Vector testVec2;

  int signSwitch = 2*(RotationVector::Usage == kindr::rotations::RotationUsage::ACTIVE)-1;

  // Check rotation of base vectors around main axis
  testVec = this->rotRotationVectorQuarterX.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-4);
  testVec = this->rotRotationVectorQuarterX.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),1e-4);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),1e-4);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),1e-4);
  testVec = this->rotRotationVectorQuarterX.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),1e-4);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),1e-4);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),1e-4);
  testVec = this->rotRotationVectorQuarterY.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),1e-4);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),1e-4);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),1e-4);
  testVec = this->rotRotationVectorQuarterY.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-4);
  testVec = this->rotRotationVectorQuarterY.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),1e-4);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),1e-4);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),1e-4);
  testVec = this->rotRotationVectorQuarterZ.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),1e-4);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),1e-4);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),1e-4);
  testVec = this->rotRotationVectorQuarterZ.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),1e-4);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),1e-4);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),1e-4);
  testVec = this->rotRotationVectorQuarterZ.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-4);

  testVec = this->rotRotationVectorQuarterX.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-4);
  testVec = this->rotRotationVectorQuarterX.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),1e-4);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),1e-4);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),1e-4);
  testVec = this->rotRotationVectorQuarterX.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),1e-4);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),1e-4);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),1e-4);
  testVec = this->rotRotationVectorQuarterY.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),1e-4);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),1e-4);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),1e-4);
  testVec = this->rotRotationVectorQuarterY.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-4);
  testVec = this->rotRotationVectorQuarterY.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),1e-4);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),1e-4);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),1e-4);
  testVec = this->rotRotationVectorQuarterZ.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),1e-4);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),1e-4);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),1e-4);
  testVec = this->rotRotationVectorQuarterZ.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),1e-4);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),1e-4);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),1e-4);
  testVec = this->rotRotationVectorQuarterZ.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-4);

  // Check rotation with Identity
  testVec = this->rotRotationVectorIdentity.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-4);
  testVec = this->rotRotationVectorIdentity.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-4);
  testVec = this->rotRotationVectorIdentity.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-4);
  testVec = this->rotRotationVectorIdentity.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-4);
  testVec = this->rotRotationVectorIdentity.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-4);
  testVec = this->rotRotationVectorIdentity.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-4);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-4);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-4);

  // Check combination between concatenation and rotate
  testVec1 = this->rotRotationVector2.rotate(this->rotRotationVector1.rotate(this->vec));
  testVec2 = (this->rotRotationVector2*this->rotRotationVector1).rotate(this->vec);
  ASSERT_NEAR(testVec1(0), testVec2(0),1e-4);
  ASSERT_NEAR(testVec1(1), testVec2(1),1e-4);
  ASSERT_NEAR(testVec1(2), testVec2(2),1e-4);
}


/* Test Exponential and Logarithmic Map
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(RotationVectorSingleTest, testMaps){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  RotationVector rot;
  Vector testVec;

  testVec = this->rotRotationVectorIdentity.getLogarithmicMap();
  ASSERT_NEAR(testVec(0), 0.0,1e-6);
  ASSERT_NEAR(testVec(1), 0.0,1e-6);
  ASSERT_NEAR(testVec(2), 0.0,1e-6);

  testVec = this->rotRotationVector1.getLogarithmicMap();
  rot.setExponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVector1.toImplementation(), rot.toImplementation(), 1e-4, "maps");

  testVec = this->rotRotationVector2.getLogarithmicMap();
  rot.setExponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVector2.toImplementation(), rot.toImplementation(), 1e-4, "maps");

  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rot.setExponentialMap(testVec);
  ASSERT_NEAR(rot.getDisparityAngle(this->rotRotationVectorIdentity),norm,1e-6);

  testVec.setZero();
  rot.setExponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationVectorIdentity.toImplementation(), rot.toImplementation(), 1e-4, "maps");

}


/*  Test Box Operations
 * Assumes isNear() of Angle Axis is correct.
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(RotationVectorSingleTest, testBoxOperators){
  typedef typename TestFixture::RotationVector RotationVector;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  RotationVector rot;
  RotationVector rot2;
  Vector testVec;

  // Test addition with 0
  testVec.setZero();
  rot = this->rotRotationVector1.boxPlus(testVec);
  ASSERT_EQ(rot.isNear(this->rotRotationVector1,1e-6),true);

  // Test subtraction of same elements
  testVec = this->rotRotationVector1.boxMinus(this->rotRotationVector1);
  ASSERT_NEAR(testVec(0),0.0,1e-6);
  ASSERT_NEAR(testVec(1),0.0,1e-6);
  ASSERT_NEAR(testVec(2),0.0,1e-6);

  // Test backward-forward
  testVec = this->rotRotationVector1.boxMinus(this->rotRotationVector2);
  rot = this->rotRotationVector2.boxPlus(testVec);
  ASSERT_EQ(rot.isNear(this->rotRotationVector1,1e-6),true);

  // Test forward-backward
  testVec = this->vec;
  rot = this->rotRotationVector1.boxPlus(testVec);
  testVec = rot.boxMinus(this->rotRotationVector1);
  ASSERT_NEAR(testVec(0),this->vec(0),1e-4);
  ASSERT_NEAR(testVec(1),this->vec(1),1e-4);
  ASSERT_NEAR(testVec(2),this->vec(2),1e-4);

  // Test overlap with disparity angle
  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rot = this->rotRotationVector1.boxPlus(testVec);
  ASSERT_NEAR(rot.getDisparityAngle(this->rotRotationVector1),norm,1e-4); // Check distance between both
  rot2 = this->rotRotationVector1.boxPlus(2*testVec);
  ASSERT_NEAR(rot.getDisparityAngle(rot2),norm,1e-4); // Check distance to double
  rot2 = this->rotRotationVector1.boxPlus(-testVec);
  ASSERT_NEAR(rot.getDisparityAngle(rot2),2*norm,1e-4); // Check distance to reverse
}


// --------------------------------------------------------------------------------------------------- //
// -------- Testing for casting between different type of rotations and rotation Quaternions --------- //
// --------------------------------------------------------------------------------------------------- //

// Test conversion between rotation quaternion and rotation vectors
TYPED_TEST(RotationQuaternionRotationVectorPairTest, testConversionRotationQuaternionRotationVector){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationVector RotationVector;
  RotationQuaternion rotQuat;
  RotationVector rotVec;

  // TODO: add generic

  rotQuat = this->rotRotationVectorIdentity;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-6);
  rotQuat = this->rotRotationVectorQuarterX;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-6);
  rotQuat = this->rotRotationVectorQuarterY;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-6);
  rotQuat = this->rotRotationVectorQuarterZ;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-6);

  rotVec = this->rotQuatIdentity;
  ASSERT_NEAR(rotVec.x(), this->rotRotationVectorIdentity.x(),1e-6);
  ASSERT_NEAR(rotVec.y(), this->rotRotationVectorIdentity.y(),1e-6);
  ASSERT_NEAR(rotVec.z(), this->rotRotationVectorIdentity.z(),1e-6);
  rotVec = this->rotQuatQuarterX;
  ASSERT_NEAR(rotVec.x(), this->rotRotationVectorQuarterX.x(),1e-6);
  ASSERT_NEAR(rotVec.y(), this->rotRotationVectorQuarterX.y(),1e-6);
  ASSERT_NEAR(rotVec.z(), this->rotRotationVectorQuarterX.z(),1e-6);
  rotVec = this->rotQuatQuarterY;
  ASSERT_NEAR(rotVec.x(), this->rotRotationVectorQuarterY.x(),1e-6);
  ASSERT_NEAR(rotVec.y(), this->rotRotationVectorQuarterY.y(),1e-6);
  ASSERT_NEAR(rotVec.z(), this->rotRotationVectorQuarterY.z(),1e-6);
  rotVec = this->rotQuatQuarterZ;
  ASSERT_NEAR(rotVec.x(), this->rotRotationVectorQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotVec.y(), this->rotRotationVectorQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotVec.z(), this->rotRotationVectorQuarterZ.z(),1e-6);
}



// --------------------------------------------------------------------------------------------------- //
// ------------------------------------- Testing Rotation Vector ------------------------------------- //
// --------------------------------------------------------------------------------------------------- //

// Test Rotation Vector Inversion
TYPED_TEST(RotationQuaternionRotationVectorPairTest, testInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationVector RotationVector;
  RotationQuaternion rotQuat;
  RotationVector rot1;
  RotationVector rot2;

  // Use rotation quaternion method as reference
  rotQuat = this->rotQuat1.inverted();
  rot1 = rotQuat;

  // Use rotation vector method and compare
  rot2 = this->rotQuat1;
  rot2.invert();
  ASSERT_NEAR(rot1.x(),rot2.x(),1e-6);
  ASSERT_NEAR(rot1.y(),rot2.y(),1e-6);
  ASSERT_NEAR(rot1.z(),rot2.z(),1e-6);
}

// TODO: do the same for all other types of rotation and all other methods

