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

namespace rot = kindr;
namespace quat = kindr;

using namespace kindr;

template <typename Rotation_>
class EulerAnglesXyzSingleTest : public ::testing::Test{
 public:
  typedef Rotation_ EulerAnglesXyz;
  typedef typename Rotation_::Scalar Scalar;

  // Eigen::Matrix 3x1
  typedef Eigen::Matrix<Scalar,3,1> Vector;

  // Rotation Quaternion
  typedef typename rot::RotationQuaternion<Scalar> RotationQuaternion;

  // Eigen::Matrix
  const Vector eigenVector3Identity = Vector(0.0,0.0,0.0);
  const Vector eigenVector3v1 = Vector(0.36,0.48,0.8);
  const Vector eigenVector3v2 = Vector(0.3,2.0,0.0);

  const Vector vec = Vector(0.3,-1.5,0.6);
  const Vector vecX = Vector(1.0,0.0,0.0);
  const Vector vecY = Vector(0.0,1.0,0.0);
  const Vector vecZ = Vector(0.0,0.0,1.0);
  const Vector vecXSmallError = Vector(1.0000001,0.0,0.0);


  // Rotation from Eigen::Matrix
  const EulerAnglesXyz rotEulerAnglesXyzV1 = EulerAnglesXyz(eigenVector3v1);
  const EulerAnglesXyz rotEulerAnglesXyzV2 = EulerAnglesXyz(eigenVector3v2);

  // Rotation from RotationQuaternion
  const EulerAnglesXyz rotEulerAnglesXyzV3 = EulerAnglesXyz(RotationQuaternion(0.0,0.36,0.48,0.8));
  const EulerAnglesXyz rotEulerAnglesXyzV4 = EulerAnglesXyz(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)));


  // Rotation
  const EulerAnglesXyz rotEulerAnglesXyzQuarterX = EulerAnglesXyz(M_PI/2.0,0.0,0.0);
  const EulerAnglesXyz rotEulerAnglesXyzQuarterY = EulerAnglesXyz(0.0,M_PI/2.0,0.0);
  const EulerAnglesXyz rotEulerAnglesXyzQuarterZ = EulerAnglesXyz(0.0,0.0,M_PI/2.0);
  const EulerAnglesXyz rotEulerAnglesXyzIdentity = EulerAnglesXyz(0.0,0.0,0.0);


};

template <typename RotationQuaternionEulerAnglesXyzImplementationPair>
struct EulerAnglesXyzRotationQuaternionPairTest : public ::testing::Test{
  typedef typename RotationQuaternionEulerAnglesXyzImplementationPair::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationQuaternionEulerAnglesXyzImplementationPair::second_type EulerAnglesXyz;
  typedef typename EulerAnglesXyz::Scalar EulerAnglesXyzScalar;

  const RotationQuaternionScalar rotQuatSmallNumber = kindr::internal::NumTraits<RotationQuaternionScalar>::dummy_precision()/10.0;
  const RotationQuaternionScalar rotEulerAnglesXyzSmallNumber = kindr::internal::NumTraits<EulerAnglesXyzScalar>::dummy_precision()/10.0;


  const RotationQuaternion rotQuatQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuatQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuatQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(1.0,0.0,0.0,0.0);
  const RotationQuaternion rotQuat1 = RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
  const RotationQuaternion rotQuat1Conj = RotationQuaternion(4.0/sqrt(30.0),-3.0/sqrt(30.0),-1.0/sqrt(30.0),-2.0/sqrt(30.0));
//  const RotationQuaternion rotQuat2 = RotationQuaternion(rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber));

  const EulerAnglesXyz rotEulerAnglesXyzQuarterX = EulerAnglesXyz(M_PI/2.0,0.0,0.0);
  const EulerAnglesXyz rotEulerAnglesXyzQuarterY = EulerAnglesXyz(0.0,M_PI/2.0,0.0);
  const EulerAnglesXyz rotEulerAnglesXyzQuarterZ = EulerAnglesXyz(0.0,0.0,M_PI/2.0);
  const EulerAnglesXyz rotEulerAnglesXyzIdentity = EulerAnglesXyz(0.0,0.0,0.0);
//  const EulerAnglesXyz rotEulerAnglesXyzV2 = EulerAnglesXyz(2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber));
//  const EulerAnglesXyz rotEulerAnglesXyz3 = EulerAnglesXyz(rotEulerAnglesXyzSmallNumber,rotEulerAnglesXyzSmallNumber,rotEulerAnglesXyzSmallNumber);
};

template <typename ImplementationPairs_>
struct EulerAnglesXyzActiveTest : public EulerAnglesXyzRotationQuaternionPairTest<ImplementationPairs_>{

};

template <typename ImplementationPairs_>
struct EulerAnglesXyzPassiveTest : public EulerAnglesXyzRotationQuaternionPairTest<ImplementationPairs_>{
};


typedef ::testing::Types<
    rot::EulerAnglesXyzPD,
    rot::EulerAnglesXyzPF
> EulerAnglesXyzTypes;


typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPD, rot::EulerAnglesXyzPD>,
    std::pair<rot::RotationQuaternionPF, rot::EulerAnglesXyzPF>
> EulerAnglesXyzPassiveTypes;



typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPF, rot::EulerAnglesXyzPF>,
    std::pair<rot::RotationQuaternionPF, rot::EulerAnglesXyzPD>,
    std::pair<rot::RotationQuaternionPD, rot::EulerAnglesXyzPF>,
    std::pair<rot::RotationQuaternionPD, rot::EulerAnglesXyzPD>
> TypeQuaternionEulerAnglesXyzPairs;

TYPED_TEST_CASE(EulerAnglesXyzSingleTest, EulerAnglesXyzTypes);
TYPED_TEST_CASE(EulerAnglesXyzRotationQuaternionPairTest, TypeQuaternionEulerAnglesXyzPairs);
TYPED_TEST_CASE(EulerAnglesXyzPassiveTest, EulerAnglesXyzPassiveTypes);



// --------------------------------------------------------------------------------------------------- //
// ------------------ Testing for constructors and getters for other rotation types ------------------ //
// --------------------------------------------------------------------------------------------------- //

// Testing constructors and getters for Rotation Vector
TYPED_TEST(EulerAnglesXyzSingleTest, testConstructors){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesXyz rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3Identity, rot.toImplementation(), Scalar(1e-4), "constructor");

  EulerAnglesXyz rot2(this->eigenVector3v1.x(),this->eigenVector3v1.y(),this->eigenVector3v1.z());
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot2.toImplementation(), Scalar(1e-4), "constructor");

  EulerAnglesXyz rot3(this->eigenVector3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot3.toImplementation(), Scalar(1e-4), "constructor");

  EulerAnglesXyz rot4(this->rotEulerAnglesXyzV1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot4.toImplementation(), Scalar(1e-4), "constructor");

  EulerAnglesXyz rot5(rot4);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot5.toImplementation(), Scalar(1e-4), "constructor");
}

TYPED_TEST(EulerAnglesXyzSingleTest, testGetters)
{
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesXyz rot(this->eigenVector3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot.vector(), Scalar(1e-4), "vector()");
  ASSERT_NEAR(rot.x(), this->eigenVector3v1.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenVector3v1.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenVector3v1.z(),1e-6);
  ASSERT_NEAR(rot.yaw(), this->eigenVector3v1.z(),1e-6);
  ASSERT_NEAR(rot.pitch(), this->eigenVector3v1.y(),1e-6);
  ASSERT_NEAR(rot.roll(), this->eigenVector3v1.x(),1e-6);

}

TYPED_TEST(EulerAnglesXyzSingleTest, testAssignmentOperator){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesXyz rot(this->eigenVector3v1);
  EulerAnglesXyz rot1 = rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot1.toImplementation(), Scalar(1e-4), "constructor");

}

TYPED_TEST(EulerAnglesXyzSingleTest, testParenthesisOperator) {
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesXyz rot(this->eigenVector3v1);
  EulerAnglesXyz rot1;
  rot1(rot);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot1.toImplementation(), Scalar(1e-4), "constructor");

}


TYPED_TEST(EulerAnglesXyzSingleTest, testSetters)
{
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesXyz rot(this->eigenVector3v1);
  rot.setIdentity();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3Identity, rot.toImplementation(), Scalar(1e-4), "identity");

  rot.setFromVectors(this->vec, this->vec);
  ASSERT_NEAR(rot.x(), this->rotEulerAnglesXyzIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotEulerAnglesXyzIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotEulerAnglesXyzIdentity.z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecY);
  ASSERT_NEAR(rot.x(), this->rotEulerAnglesXyzQuarterZ.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotEulerAnglesXyzQuarterZ.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotEulerAnglesXyzQuarterZ.z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecX);
  ASSERT_NEAR(rot.x(), this->rotEulerAnglesXyzIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotEulerAnglesXyzIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotEulerAnglesXyzIdentity.z(),1e-6);


  rot.setFromVectors(this->vecX, this->vecXSmallError);
  ASSERT_NEAR(rot.x(), this->rotEulerAnglesXyzIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotEulerAnglesXyzIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotEulerAnglesXyzIdentity.z(),1e-6);
}

/* Test Uniqueness
 *  Assumes conversion between rotation quaternion and rotation vector is correct.
 */
TYPED_TEST(EulerAnglesXyzSingleTest, testUniqueness){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesXyz rotEulerAnglesXyz;
  EulerAnglesXyz rotEulerAnglesXyzV2;
  EulerAnglesXyz rotEulerAnglesXyzUnique;

  // Check uniqueness getter and setter with generic Rotation Quaternions
  rotEulerAnglesXyz = RotationQuaternion(-1.0,0.0,0.0,0.0);
  rotEulerAnglesXyzUnique =  RotationQuaternion(1.0,0.0,0.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyz.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesXyz = RotationQuaternion(0.0,-1.0,0.0,0.0);
  rotEulerAnglesXyzUnique = RotationQuaternion(0.0,1.0,0.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyz.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesXyz = RotationQuaternion(0.0,0.0,-1.0,0.0);
  rotEulerAnglesXyzUnique = RotationQuaternion(0.0,0.0,1.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyz.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesXyz = RotationQuaternion(0.0,0.0,0.0,1.0);
  rotEulerAnglesXyzUnique = RotationQuaternion(0.0,0.0,0.0,-1.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyz.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesXyz = RotationQuaternion(1.0,0.0,0.0,0.0);
  rotEulerAnglesXyzUnique = RotationQuaternion(-1.0,0.0,0.0,0.0);
  rotEulerAnglesXyzV2 = rotEulerAnglesXyz.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyz.toImplementation(), 2*M_PI, 1e-2, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyzV2.toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesXyz = RotationQuaternion(0.0,1.0,0.0,0.0);
  rotEulerAnglesXyzUnique = RotationQuaternion(0.0,-1.0,0.0,0.0);
  rotEulerAnglesXyzV2 = rotEulerAnglesXyz.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyz.toImplementation(), 2*M_PI, 1e-2, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyzV2.toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesXyz = RotationQuaternion(0.0,0.0,1.0,0.0);
  rotEulerAnglesXyzUnique = RotationQuaternion(0.0,0.0,-1.0,0.0);
  rotEulerAnglesXyzV2 = rotEulerAnglesXyz.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyz.toImplementation(), 2*M_PI, 1e-2, "unique");


  rotEulerAnglesXyz = RotationQuaternion(0.0,0.0,0.0,1.0);
  rotEulerAnglesXyzUnique = RotationQuaternion(0.0,0.0,0.0,-1.0);
  rotEulerAnglesXyzV2 = rotEulerAnglesXyz.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyz.toImplementation(), 2*M_PI, 1e-2, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesXyzUnique.getUnique().toImplementation(), rotEulerAnglesXyzV2.toImplementation(), 2*M_PI, 1e-2, "unique");

}

/* Test comparison (equality)
 *
 */
TYPED_TEST(EulerAnglesXyzSingleTest, testComparisonEqual){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesXyz rot;

  // Check equality comparison
  rot = this->rotEulerAnglesXyzV1;
  ASSERT_EQ(true, rot==this->rotEulerAnglesXyzV1);
  ASSERT_EQ(false, rot==this->rotEulerAnglesXyzV2);
}

/* Test comparison (inequality)
 *
 */
TYPED_TEST(EulerAnglesXyzSingleTest, testComparisonNotEqual){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesXyz rot;

  // Check inequality comparison
  rot = this->rotEulerAnglesXyzV1;
  ASSERT_EQ(false, rot!=this->rotEulerAnglesXyzV1);
  ASSERT_EQ(true, rot!=this->rotEulerAnglesXyzV2);
}

/* Test  getDisparityAngle
 * Assumes conversion between EulerAnglesXyz and RotationQuaternion is correct.
 */
TYPED_TEST(EulerAnglesXyzSingleTest, testGetDisparityAngle){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  ASSERT_NEAR(this->rotEulerAnglesXyzV3.getDisparityAngle(this->rotEulerAnglesXyzV3),0.0,1e-6);
  ASSERT_NEAR(this->rotEulerAnglesXyzV4.getDisparityAngle(this->rotEulerAnglesXyzV4),0.0,1e-6);
  ASSERT_NEAR(this->rotEulerAnglesXyzIdentity.getDisparityAngle(this->rotEulerAnglesXyzIdentity),0.0,1e-6);
  ASSERT_NEAR(this->rotEulerAnglesXyzV4.getDisparityAngle(this->rotEulerAnglesXyzV3),fabs(this->rotEulerAnglesXyzV3.getDisparityAngle(this->rotEulerAnglesXyzV4)),1e-6);
  ASSERT_NEAR(this->rotEulerAnglesXyzV3.getDisparityAngle(this->rotEulerAnglesXyzIdentity),calcRotationQuatDisparityAngleToIdentity(RotationQuaternion(this->rotEulerAnglesXyzV3)),1e-6);
  ASSERT_NEAR(this->rotEulerAnglesXyzV4.getDisparityAngle(this->rotEulerAnglesXyzV3),calcRotationQuatDisparityAngle(RotationQuaternion(this->rotEulerAnglesXyzV3), RotationQuaternion(this->rotEulerAnglesXyzV4)),1e-6);
}

/* Test isNear
 * Assumes that getDisparityAngle() is correct.
 */
TYPED_TEST(EulerAnglesXyzSingleTest, testIsNear){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesXyz rot;

  // Check isNear
  rot = this->rotEulerAnglesXyzV1;
  ASSERT_EQ(rot.isNear(this->rotEulerAnglesXyzV1,1e-6),true);
  ASSERT_EQ(rot.isNear(this->rotEulerAnglesXyzV2,1e-6),false);
}



/* Test Concatenation
 * Assumes isNear is correct.
 */
TYPED_TEST(EulerAnglesXyzSingleTest, testConcatenation){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesXyz rotEulerAnglesXyz;

  // Check result of multiplication of a generic rotation with identity
  rotEulerAnglesXyz = this->rotEulerAnglesXyzV3*this->rotEulerAnglesXyzIdentity;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzV3,1e-6),true);
  rotEulerAnglesXyz = this->rotEulerAnglesXyzIdentity*this->rotEulerAnglesXyzV3;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzV3,1e-6),true);

  // Check concatenation of 4 quarters
  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterX*this->rotEulerAnglesXyzQuarterX*this->rotEulerAnglesXyzQuarterX*this->rotEulerAnglesXyzQuarterX;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzIdentity,1e-6),true);

  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterY*this->rotEulerAnglesXyzQuarterY*this->rotEulerAnglesXyzQuarterY*this->rotEulerAnglesXyzQuarterY;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzIdentity,1e-6),true);

  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterZ*this->rotEulerAnglesXyzQuarterZ*this->rotEulerAnglesXyzQuarterZ*this->rotEulerAnglesXyzQuarterZ;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzIdentity,1e-6),true);

  // Check concatenation of 3 different quarters
  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterX.inverted()*this->rotEulerAnglesXyzQuarterY*this->rotEulerAnglesXyzQuarterX;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzQuarterZ.inverted(),1e-6),true);

  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterX.inverted()*this->rotEulerAnglesXyzQuarterZ*this->rotEulerAnglesXyzQuarterX;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzQuarterY,1e-6),true);

  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterY.inverted()*this->rotEulerAnglesXyzQuarterX*this->rotEulerAnglesXyzQuarterY;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzQuarterZ,1e-6),true);

  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterY.inverted()*this->rotEulerAnglesXyzQuarterZ*this->rotEulerAnglesXyzQuarterY;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzQuarterX.inverted(),1e-6),true);

  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterZ.inverted()*this->rotEulerAnglesXyzQuarterX*this->rotEulerAnglesXyzQuarterZ;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzQuarterY.inverted(),1e-6),true);

  rotEulerAnglesXyz = this->rotEulerAnglesXyzQuarterZ.inverted()*this->rotEulerAnglesXyzQuarterY*this->rotEulerAnglesXyzQuarterZ;
  ASSERT_EQ(rotEulerAnglesXyz.isNear(this->rotEulerAnglesXyzQuarterX,1e-6),true);
}


// Test fix
TYPED_TEST(EulerAnglesXyzSingleTest, testFix){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesXyz rot;
  EulerAnglesXyz rotModified;

  // Check fix
  rot = this->rotEulerAnglesXyzV1;
  rotModified = rot;
//  rotModified.toImplementation() *= 1.1; // nothing to be fixed, just checking the function
  rotModified.fix();
  ASSERT_NEAR(rotModified.x(), rot.x(),1e-6);
  ASSERT_NEAR(rotModified.y(), rot.y(),1e-6);
  ASSERT_NEAR(rotModified.z(), rot.z(),1e-6);
}


// Test Vector Rotation
TYPED_TEST(EulerAnglesXyzSingleTest, testVectorRotation){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;
  Vector testVec1;
  Vector testVec2;

  int signSwitch = 1;

  // Check rotation of base vectors around main axis
  testVec = this->rotEulerAnglesXyzQuarterX.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterX.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterX.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterY.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterY.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterY.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterZ.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterZ.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterZ.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecZ(2),Scalar(1e-4));

  testVec = this->rotEulerAnglesXyzQuarterX.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterX.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterX.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterY.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterY.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterY.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterZ.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterZ.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzQuarterZ.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecZ(2),Scalar(1e-4));

  // Check rotation with Identity
  testVec = this->rotEulerAnglesXyzIdentity.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzIdentity.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzIdentity.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzIdentity.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzIdentity.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesXyzIdentity.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecZ(2),Scalar(1e-4));

  // Check combination between concatenation and rotate
  testVec1 = this->rotEulerAnglesXyzV4.rotate(this->rotEulerAnglesXyzV3.rotate(this->vec));
  testVec2 = (this->rotEulerAnglesXyzV4*this->rotEulerAnglesXyzV3).rotate(this->vec);
  ASSERT_NEAR(testVec1(0), testVec2(0),Scalar(1e-4));
  ASSERT_NEAR(testVec1(1), testVec2(1),Scalar(1e-4));
  ASSERT_NEAR(testVec1(2), testVec2(2),Scalar(1e-4));
}




/* Test Exponential and Logarithmic Map
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(EulerAnglesXyzSingleTest, testMaps){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;

  testVec = this->rotEulerAnglesXyzIdentity.logarithmicMap();
  ASSERT_NEAR(testVec(0), 0.0,1e-6);
  ASSERT_NEAR(testVec(1), 0.0,1e-6);
  ASSERT_NEAR(testVec(2), 0.0,1e-6);

  testVec = this->rotEulerAnglesXyzV3.logarithmicMap();
  EulerAnglesXyz rotExpMap = EulerAnglesXyz::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotEulerAnglesXyzV3.toImplementation(), rotExpMap.toImplementation(), Scalar(1e-4), "maps");

  testVec = this->rotEulerAnglesXyzV4.logarithmicMap();
  rotExpMap =  EulerAnglesXyz::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotEulerAnglesXyzV4.toImplementation(), rotExpMap.toImplementation(), Scalar(1e-4), "maps");

  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rotExpMap = EulerAnglesXyz::exponentialMap(testVec);
  ASSERT_NEAR(rotExpMap.getDisparityAngle(this->rotEulerAnglesXyzIdentity),norm,1e-6);

  testVec.setZero();
  rotExpMap = EulerAnglesXyz::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotEulerAnglesXyzIdentity.toImplementation(), rotExpMap.toImplementation(), Scalar(1e-4), "maps");

}


/*  Test Box Operations
 * Assumes isNear() of Angle Axis is correct.
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(EulerAnglesXyzSingleTest, testBoxOperators){
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  EulerAnglesXyz rot;
  EulerAnglesXyz rot2;
  Vector testVec;

  // Test addition with 0
  testVec.setZero();
  rot = this->rotEulerAnglesXyzV3.boxPlus(testVec);
  ASSERT_EQ(rot.isNear(this->rotEulerAnglesXyzV3,1e-6),true);

  // Test subtraction of same elements
  testVec = this->rotEulerAnglesXyzV3.boxMinus(this->rotEulerAnglesXyzV3);
  ASSERT_NEAR(testVec(0),0.0,1e-6);
  ASSERT_NEAR(testVec(1),0.0,1e-6);
  ASSERT_NEAR(testVec(2),0.0,1e-6);

  // Test backward-forward
  testVec = this->rotEulerAnglesXyzV3.boxMinus(this->rotEulerAnglesXyzV4);
  rot = this->rotEulerAnglesXyzV4.boxPlus(testVec);
  ASSERT_EQ(rot.isNear(this->rotEulerAnglesXyzV3,1e-6),true);

  // Test forward-backward
  testVec = this->vec;
  rot = this->rotEulerAnglesXyzV3.boxPlus(testVec);
  testVec = rot.boxMinus(this->rotEulerAnglesXyzV3);
  ASSERT_NEAR(testVec(0),this->vec(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1),this->vec(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2),this->vec(2),Scalar(1e-4));

  // Test overlap with disparity angle
  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rot = this->rotEulerAnglesXyzV3.boxPlus(testVec);
  ASSERT_NEAR(rot.getDisparityAngle(this->rotEulerAnglesXyzV3),norm,Scalar(1e-4)); // Check distance between both
  rot2 = this->rotEulerAnglesXyzV3.boxPlus(2*testVec);
  ASSERT_NEAR(rot.getDisparityAngle(rot2),norm,Scalar(1e-4)); // Check distance to double
  rot2 = this->rotEulerAnglesXyzV3.boxPlus(-testVec);
  ASSERT_NEAR(rot.getDisparityAngle(rot2),2*norm,Scalar(1e-4)); // Check distance to reverse
}


// --------------------------------------------------------------------------------------------------- //
// -------- Testing for casting between different type of rotations and rotation Quaternions --------- //
// --------------------------------------------------------------------------------------------------- //

// Test conversion between rotation quaternion and rotation vectors
TYPED_TEST(EulerAnglesXyzRotationQuaternionPairTest, testConversionRotationQuaternionEulerAnglesXyz){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  RotationQuaternion rotQuat;
  EulerAnglesXyz rotEulerAnglesXyz;

  // TODO: add generic

  rotQuat = this->rotEulerAnglesXyzIdentity;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-3);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-3);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-3);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-3);
  rotQuat = this->rotEulerAnglesXyzQuarterX;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-3);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-3);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-3);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-3);
  rotQuat = this->rotEulerAnglesXyzQuarterY;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-3);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-3);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-3);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-3);
  rotQuat = this->rotEulerAnglesXyzQuarterZ;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-3);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-3);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-3);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-3);

  rotEulerAnglesXyz = this->rotQuatIdentity;
  ASSERT_NEAR(rotEulerAnglesXyz.x(), this->rotEulerAnglesXyzIdentity.x(),1e-3);
  ASSERT_NEAR(rotEulerAnglesXyz.y(), this->rotEulerAnglesXyzIdentity.y(),1e-3);
  ASSERT_NEAR(rotEulerAnglesXyz.z(), this->rotEulerAnglesXyzIdentity.z(),1e-3);
  rotEulerAnglesXyz = this->rotQuatQuarterX;
  ASSERT_NEAR(rotEulerAnglesXyz.x(), this->rotEulerAnglesXyzQuarterX.x(),1e-3);
  ASSERT_NEAR(rotEulerAnglesXyz.y(), this->rotEulerAnglesXyzQuarterX.y(),1e-3);
  ASSERT_NEAR(rotEulerAnglesXyz.z(), this->rotEulerAnglesXyzQuarterX.z(),1e-3);
  rotEulerAnglesXyz = this->rotQuatQuarterY;
  ASSERT_NEAR(rotEulerAnglesXyz.x(), this->rotEulerAnglesXyzQuarterY.x(),1e-3);
  ASSERT_NEAR(rotEulerAnglesXyz.y(), this->rotEulerAnglesXyzQuarterY.y(),1e-3);
  ASSERT_NEAR(rotEulerAnglesXyz.z(), this->rotEulerAnglesXyzQuarterY.z(),1e-3);
  rotEulerAnglesXyz = this->rotQuatQuarterZ;
  ASSERT_NEAR(rotEulerAnglesXyz.x(), this->rotEulerAnglesXyzQuarterZ.x(),1e-3);
  ASSERT_NEAR(rotEulerAnglesXyz.y(), this->rotEulerAnglesXyzQuarterZ.y(),1e-3);
  ASSERT_NEAR(rotEulerAnglesXyz.z(), this->rotEulerAnglesXyzQuarterZ.z(),1e-3);
}


// Test Inversion
TYPED_TEST(EulerAnglesXyzRotationQuaternionPairTest, testInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  RotationQuaternion rotQuat;
  EulerAnglesXyz rot1;
  EulerAnglesXyz rot2;

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

TYPED_TEST(EulerAnglesXyzSingleTest, testRotationOrder)
{
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;

  EulerAnglesXyz rot1(this->eigenVector3v1);
  EulerAnglesXyz rot2 = EulerAnglesXyz(this->eigenVector3v1(0),0,0)
                        *EulerAnglesXyz(0,this->eigenVector3v1(1),0)
                        *EulerAnglesXyz(0.0, 0.0, this->eigenVector3v1(2));

  ASSERT_NEAR(rot1.x(), rot2.x(),1e-6);
  ASSERT_NEAR(rot1.y(), rot2.y(),1e-6);
  ASSERT_NEAR(rot1.z(), rot2.z(),1e-6);

}


TYPED_TEST(EulerAnglesXyzSingleTest, testRotationMatrix)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::EulerAnglesXyz EulerAnglesXyz;

  Scalar x = M_PI_4;
  Scalar y = 1.2;
  Scalar z = -0.8;
  kindr::RotationMatrix<Scalar> rotMatKindr(EulerAnglesXyz(x, y, z));

  using std::cos;
  using std::sin;
  Eigen::Matrix<Scalar, 3, 3> rotMat;

  rotMat <<                         cos(y)*cos(z),                       -cos(y)*sin(z),         sin(y),
   cos(x)*sin(z) + cos(z)*sin(x)*sin(y), cos(x)*cos(z) - sin(x)*sin(y)*sin(z), -cos(y)*sin(x),
   sin(x)*sin(z) - cos(x)*cos(z)*sin(y), cos(z)*sin(x) + cos(x)*sin(y)*sin(z),  cos(x)*cos(y);
  KINDR_ASSERT_DOUBLE_MX_EQ(rotMat, rotMatKindr.matrix(), Scalar(1.0e-3), "rotation matrix")
}

