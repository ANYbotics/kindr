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
class EulerAnglesZyxSingleTest : public ::testing::Test{
 public:
  typedef Rotation_ EulerAnglesZyx;
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
  const EulerAnglesZyx rotEulerAnglesZyxV1 = EulerAnglesZyx(eigenVector3v1);
  const EulerAnglesZyx rotEulerAnglesZyxV2 = EulerAnglesZyx(eigenVector3v2);

  // Rotation from RotationQuaternion
  const EulerAnglesZyx rotEulerAnglesZyxV3 = EulerAnglesZyx(RotationQuaternion(0.0,0.36,0.48,0.8));
  const EulerAnglesZyx rotEulerAnglesZyxV4 = EulerAnglesZyx(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)));


  // Rotation
  const EulerAnglesZyx rotEulerAnglesZyxQuarterX = EulerAnglesZyx(0.0,0.0,M_PI/2.0);
  const EulerAnglesZyx rotEulerAnglesZyxQuarterY = EulerAnglesZyx(0.0,M_PI/2.0,0.0);
  const EulerAnglesZyx rotEulerAnglesZyxQuarterZ = EulerAnglesZyx(M_PI/2.0,0.0,0.0);
  const EulerAnglesZyx rotEulerAnglesZyxIdentity = EulerAnglesZyx(0.0,0.0,0.0);


};

template <typename RotationQuaternionEulerAnglesZyxImplementationPair>
struct EulerAnglesZyxRotationQuaternionPairTest : public ::testing::Test{
  typedef typename RotationQuaternionEulerAnglesZyxImplementationPair::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationQuaternionEulerAnglesZyxImplementationPair::second_type EulerAnglesZyx;
  typedef typename EulerAnglesZyx::Scalar EulerAnglesZyxScalar;

  const RotationQuaternionScalar rotQuatSmallNumber = kindr::internal::NumTraits<RotationQuaternionScalar>::dummy_precision()/10.0;
  const RotationQuaternionScalar rotEulerAnglesZyxSmallNumber = kindr::internal::NumTraits<EulerAnglesZyxScalar>::dummy_precision()/10.0;


  const RotationQuaternion rotQuatQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuatQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuatQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(1.0,0.0,0.0,0.0);
  const RotationQuaternion rotQuat1 = RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
  const RotationQuaternion rotQuat1Conj = RotationQuaternion(4.0/sqrt(30.0),-3.0/sqrt(30.0),-1.0/sqrt(30.0),-2.0/sqrt(30.0));
//  const RotationQuaternion rotQuat2 = RotationQuaternion(rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber));

  const EulerAnglesZyx rotEulerAnglesZyxQuarterX = EulerAnglesZyx(0.0,0.0,M_PI/2.0);
  const EulerAnglesZyx rotEulerAnglesZyxQuarterY = EulerAnglesZyx(0.0,M_PI/2.0,0.0);
  const EulerAnglesZyx rotEulerAnglesZyxQuarterZ = EulerAnglesZyx(M_PI/2.0,0.0,0.0);
  const EulerAnglesZyx rotEulerAnglesZyxIdentity = EulerAnglesZyx(0.0,0.0,0.0);
//  const EulerAnglesZyx rotEulerAnglesZyxV2 = EulerAnglesZyx(2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber),2.0*rotQuatSmallNumber/sqrt(4.0*rotQuatSmallNumber*rotQuatSmallNumber));
//  const EulerAnglesZyx rotEulerAnglesZyx3 = EulerAnglesZyx(rotEulerAnglesZyxSmallNumber,rotEulerAnglesZyxSmallNumber,rotEulerAnglesZyxSmallNumber);
};

template <typename ImplementationPairs_>
struct EulerAnglesZyxActiveTest : public EulerAnglesZyxRotationQuaternionPairTest<ImplementationPairs_>{

};

template <typename ImplementationPairs_>
struct EulerAnglesZyxPassiveTest : public EulerAnglesZyxRotationQuaternionPairTest<ImplementationPairs_>{
};


typedef ::testing::Types<
    rot::EulerAnglesZyxPD,
    rot::EulerAnglesZyxPF
> EulerAnglesZyxTypes;

typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPD, rot::EulerAnglesZyxPD>,
    std::pair<rot::RotationQuaternionPF, rot::EulerAnglesZyxPF>
> EulerAnglesZyxPassiveTypes;



typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPF, rot::EulerAnglesZyxPF>,
    std::pair<rot::RotationQuaternionPF, rot::EulerAnglesZyxPD>,
    std::pair<rot::RotationQuaternionPD, rot::EulerAnglesZyxPF>,
    std::pair<rot::RotationQuaternionPD, rot::EulerAnglesZyxPD>
> TypeQuaternionEulerAnglesZyxPairs;

TYPED_TEST_CASE(EulerAnglesZyxSingleTest, EulerAnglesZyxTypes);
TYPED_TEST_CASE(EulerAnglesZyxRotationQuaternionPairTest, TypeQuaternionEulerAnglesZyxPairs);
TYPED_TEST_CASE(EulerAnglesZyxPassiveTest, EulerAnglesZyxPassiveTypes);



// --------------------------------------------------------------------------------------------------- //
// ------------------ Testing for constructors and getters for other rotation types ------------------ //
// --------------------------------------------------------------------------------------------------- //

// Testing constructors and getters for Rotation Vector
TYPED_TEST(EulerAnglesZyxSingleTest, testConstructors){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesZyx rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3Identity, rot.toImplementation(), Scalar(1e-4), "constructor");

  EulerAnglesZyx rot2(this->eigenVector3v1.x(),this->eigenVector3v1.y(),this->eigenVector3v1.z());
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot2.toImplementation(), Scalar(1e-4), "constructor");

  EulerAnglesZyx rot3(this->eigenVector3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot3.toImplementation(), Scalar(1e-4), "constructor");

  EulerAnglesZyx rot4(this->rotEulerAnglesZyxV1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot4.toImplementation(), Scalar(1e-4), "constructor");

  EulerAnglesZyx rot5(rot4);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot5.toImplementation(), Scalar(1e-4), "constructor");
}

TYPED_TEST(EulerAnglesZyxSingleTest, testGetters)
{
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesZyx rot(this->eigenVector3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot.vector(), Scalar(1e-4), "vector()");
  ASSERT_NEAR(rot.x(), this->eigenVector3v1.z(),1e-6);
  ASSERT_NEAR(rot.y(), this->eigenVector3v1.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->eigenVector3v1.x(),1e-6);
  ASSERT_NEAR(rot.yaw(), this->eigenVector3v1.x(),1e-6);
  ASSERT_NEAR(rot.pitch(), this->eigenVector3v1.y(),1e-6);
  ASSERT_NEAR(rot.roll(), this->eigenVector3v1.z(),1e-6);

}

TYPED_TEST(EulerAnglesZyxSingleTest, testAssignmentOperator){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesZyx rot(this->eigenVector3v1);
  EulerAnglesZyx rot1 = rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot1.toImplementation(), Scalar(1e-4), "constructor");

}

TYPED_TEST(EulerAnglesZyxSingleTest, testParenthesisOperator) {
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesZyx rot(this->eigenVector3v1);
  EulerAnglesZyx rot1;
  rot1(rot);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3v1, rot1.toImplementation(), Scalar(1e-4), "constructor");

}


TYPED_TEST(EulerAnglesZyxSingleTest, testSetters)
{
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;

  EulerAnglesZyx rot(this->eigenVector3v1);
  rot.setIdentity();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenVector3Identity, rot.toImplementation(), Scalar(1e-4), "identity");

  rot.setFromVectors(this->vec, this->vec);
  ASSERT_NEAR(rot.x(), this->rotEulerAnglesZyxIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotEulerAnglesZyxIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotEulerAnglesZyxIdentity.z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecY);
  ASSERT_NEAR(rot.x(), this->rotEulerAnglesZyxQuarterZ.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotEulerAnglesZyxQuarterZ.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotEulerAnglesZyxQuarterZ.z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecX);
  ASSERT_NEAR(rot.x(), this->rotEulerAnglesZyxIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotEulerAnglesZyxIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotEulerAnglesZyxIdentity.z(),1e-6);

  rot.setFromVectors(this->vecX, this->vecXSmallError);
  ASSERT_NEAR(rot.x(), this->rotEulerAnglesZyxIdentity.x(),1e-6);
  ASSERT_NEAR(rot.y(), this->rotEulerAnglesZyxIdentity.y(),1e-6);
  ASSERT_NEAR(rot.z(), this->rotEulerAnglesZyxIdentity.z(),1e-6);

}

/* Test Uniqueness
 *  Assumes conversion between rotation quaternion and rotation vector is correct.
 */
TYPED_TEST(EulerAnglesZyxSingleTest, testUniqueness){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesZyx rotEulerAnglesZyx;
  EulerAnglesZyx rotEulerAnglesZyxV2;
  EulerAnglesZyx rotEulerAnglesZyxUnique;

  // Check uniqueness getter and setter with generic Rotation Quaternions
  rotEulerAnglesZyx = RotationQuaternion(-1.0,0.0,0.0,0.0);
  rotEulerAnglesZyxUnique =  RotationQuaternion(1.0,0.0,0.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesZyx = RotationQuaternion(0.0,-1.0,0.0,0.0);
  rotEulerAnglesZyxUnique = RotationQuaternion(0.0,1.0,0.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesZyx = RotationQuaternion(0.0,0.0,-1.0,0.0);
  rotEulerAnglesZyxUnique = RotationQuaternion(0.0,0.0,1.0,0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesZyx = RotationQuaternion(0.0,0.0,0.0,1.0);
  rotEulerAnglesZyxUnique = RotationQuaternion(0.0,0.0,0.0,-1.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesZyx = RotationQuaternion(1.0,0.0,0.0,0.0);
  rotEulerAnglesZyxUnique = RotationQuaternion(-1.0,0.0,0.0,0.0);
  rotEulerAnglesZyxV2 = rotEulerAnglesZyx.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.toImplementation(), 2*M_PI, 1e-2, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyxV2.toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesZyx = RotationQuaternion(0.0,1.0,0.0,0.0);
  rotEulerAnglesZyxUnique = RotationQuaternion(0.0,-1.0,0.0,0.0);
  rotEulerAnglesZyxV2 = rotEulerAnglesZyx.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.toImplementation(), 2*M_PI, 1e-2, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyxV2.toImplementation(), 2*M_PI, 1e-2, "unique");

  rotEulerAnglesZyx = RotationQuaternion(0.0,0.0,1.0,0.0);
  rotEulerAnglesZyxUnique = RotationQuaternion(0.0,0.0,-1.0,0.0);
  rotEulerAnglesZyxV2 = rotEulerAnglesZyx.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.toImplementation(), 2*M_PI, 1e-2, "unique");


  rotEulerAnglesZyx = RotationQuaternion(0.0,0.0,0.0,1.0);
  rotEulerAnglesZyxUnique = RotationQuaternion(0.0,0.0,0.0,-1.0);
  rotEulerAnglesZyxV2 = rotEulerAnglesZyx.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.toImplementation(), 2*M_PI, 1e-2, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyxV2.toImplementation(), 2*M_PI, 1e-2, "unique");


  rotEulerAnglesZyx = EulerAnglesZyx(0.0, M_PI/2.0, 0.0);
  rotEulerAnglesZyxUnique =  EulerAnglesZyx(0.0, M_PI/2.0, 0.0);
  KINDR_ASSERT_DOUBLE_MX_EQ_PERIODIC(rotEulerAnglesZyxUnique.getUnique().toImplementation(), rotEulerAnglesZyx.getUnique().toImplementation(), 2*M_PI, 1e-2, "unique");

}

/* Test comparison (equality)
 *
 */
TYPED_TEST(EulerAnglesZyxSingleTest, testComparisonEqual){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesZyx rot;

  // Check equality comparison
  rot = this->rotEulerAnglesZyxV1;
  ASSERT_EQ(true, rot==this->rotEulerAnglesZyxV1);
  ASSERT_EQ(false, rot==this->rotEulerAnglesZyxV2);
}

/* Test comparison (inequality)
 *
 */
TYPED_TEST(EulerAnglesZyxSingleTest, testComparisonNotEqual){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesZyx rot;

  // Check equality comparison
  rot = this->rotEulerAnglesZyxV1;
  ASSERT_EQ(false, rot!=this->rotEulerAnglesZyxV1);
  ASSERT_EQ(true, rot!=this->rotEulerAnglesZyxV2);
}

/* Test  getDisparityAngle
 * Assumes conversion between EulerAnglesZyx and RotationQuaternion is correct.
 */
TYPED_TEST(EulerAnglesZyxSingleTest, testGetDisparityAngle){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  ASSERT_NEAR(this->rotEulerAnglesZyxV3.getDisparityAngle(this->rotEulerAnglesZyxV3),0.0,1e-6);
  ASSERT_NEAR(this->rotEulerAnglesZyxV4.getDisparityAngle(this->rotEulerAnglesZyxV4),0.0,1e-6);
  ASSERT_NEAR(this->rotEulerAnglesZyxIdentity.getDisparityAngle(this->rotEulerAnglesZyxIdentity),0.0,1e-6);
  ASSERT_NEAR(this->rotEulerAnglesZyxV4.getDisparityAngle(this->rotEulerAnglesZyxV3),fabs(this->rotEulerAnglesZyxV3.getDisparityAngle(this->rotEulerAnglesZyxV4)),1e-6);
  ASSERT_NEAR(this->rotEulerAnglesZyxV3.getDisparityAngle(this->rotEulerAnglesZyxIdentity),calcRotationQuatDisparityAngleToIdentity(RotationQuaternion(this->rotEulerAnglesZyxV3)),1e-6);
  ASSERT_NEAR(this->rotEulerAnglesZyxV4.getDisparityAngle(this->rotEulerAnglesZyxV3),calcRotationQuatDisparityAngle(RotationQuaternion(this->rotEulerAnglesZyxV3), RotationQuaternion(this->rotEulerAnglesZyxV4)),1e-6);
}

/* Test isNear
 * Assumes that getDisparityAngle() is correct.
 */
TYPED_TEST(EulerAnglesZyxSingleTest, testIsNear){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesZyx rot;

  // Check isNear
  rot = this->rotEulerAnglesZyxV1;
  ASSERT_EQ(rot.isNear(this->rotEulerAnglesZyxV1,1e-6),true);
  ASSERT_EQ(rot.isNear(this->rotEulerAnglesZyxV2,1e-6),false);
}



/* Test Concatenation
 * Assumes isNear is correct.
 */
TYPED_TEST(EulerAnglesZyxSingleTest, testConcatenation){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesZyx rotEulerAnglesZyx;

  // Check result of multiplication of a generic rotation with identity
  rotEulerAnglesZyx = this->rotEulerAnglesZyxV3*this->rotEulerAnglesZyxIdentity;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxV3,1e-6),true);
  rotEulerAnglesZyx = this->rotEulerAnglesZyxIdentity*this->rotEulerAnglesZyxV3;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxV3,1e-6),true);

  // Check concatenation of 4 quarters
  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterX*this->rotEulerAnglesZyxQuarterX*this->rotEulerAnglesZyxQuarterX*this->rotEulerAnglesZyxQuarterX;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxIdentity,1e-6),true);

  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterY*this->rotEulerAnglesZyxQuarterY*this->rotEulerAnglesZyxQuarterY*this->rotEulerAnglesZyxQuarterY;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxIdentity,1e-6),true);

  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterZ*this->rotEulerAnglesZyxQuarterZ*this->rotEulerAnglesZyxQuarterZ*this->rotEulerAnglesZyxQuarterZ;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxIdentity,1e-6),true);

  // Check concatenation of 3 different quarters
  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterX.inverted()*this->rotEulerAnglesZyxQuarterY*this->rotEulerAnglesZyxQuarterX;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxQuarterZ.inverted(),1e-6),true);

  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterX.inverted()*this->rotEulerAnglesZyxQuarterZ*this->rotEulerAnglesZyxQuarterX;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxQuarterY,1e-6),true);

  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterY.inverted()*this->rotEulerAnglesZyxQuarterX*this->rotEulerAnglesZyxQuarterY;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxQuarterZ,1e-6),true);

  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterY.inverted()*this->rotEulerAnglesZyxQuarterZ*this->rotEulerAnglesZyxQuarterY;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxQuarterX.inverted(),1e-6),true);

  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterZ.inverted()*this->rotEulerAnglesZyxQuarterX*this->rotEulerAnglesZyxQuarterZ;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxQuarterY.inverted(),1e-6),true);

  rotEulerAnglesZyx = this->rotEulerAnglesZyxQuarterZ.inverted()*this->rotEulerAnglesZyxQuarterY*this->rotEulerAnglesZyxQuarterZ;
  ASSERT_EQ(rotEulerAnglesZyx.isNear(this->rotEulerAnglesZyxQuarterX,1e-6),true);
}


// Test fix
TYPED_TEST(EulerAnglesZyxSingleTest, testFix){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  EulerAnglesZyx rot;
  EulerAnglesZyx rotModified;

  // Check fix
  rot = this->rotEulerAnglesZyxV1;
  rotModified = rot;
//  rotModified.toImplementation() *= 1.1; // nothing to be fixed, just checking the function
  rotModified.fix();
  ASSERT_NEAR(rotModified.x(), rot.x(),1e-6);
  ASSERT_NEAR(rotModified.y(), rot.y(),1e-6);
  ASSERT_NEAR(rotModified.z(), rot.z(),1e-6);
}


// Test Vector Rotation
TYPED_TEST(EulerAnglesZyxSingleTest, testVectorRotation){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;
  Vector testVec1;
  Vector testVec2;

  int signSwitch = 1;

  // Check rotation of base vectors around main axis
  testVec = this->rotEulerAnglesZyxQuarterX.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterX.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterX.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterY.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterY.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterY.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterZ.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterZ.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterZ.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecZ(2),Scalar(1e-4));

  testVec = this->rotEulerAnglesZyxQuarterX.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterX.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterX.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterY.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterY.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterY.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterZ.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterZ.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxQuarterZ.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecZ(2),Scalar(1e-4));

  // Check rotation with Identity
  testVec = this->rotEulerAnglesZyxIdentity.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxIdentity.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxIdentity.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecZ(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxIdentity.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecX(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecX(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxIdentity.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecY(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecY(2),Scalar(1e-4));
  testVec = this->rotEulerAnglesZyxIdentity.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1), this->vecZ(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2), this->vecZ(2),Scalar(1e-4));

  // Check combination between concatenation and rotate
  testVec1 = this->rotEulerAnglesZyxV4.rotate(this->rotEulerAnglesZyxV3.rotate(this->vec));
  testVec2 = (this->rotEulerAnglesZyxV4*this->rotEulerAnglesZyxV3).rotate(this->vec);
  ASSERT_NEAR(testVec1(0), testVec2(0),Scalar(1e-4));
  ASSERT_NEAR(testVec1(1), testVec2(1),Scalar(1e-4));
  ASSERT_NEAR(testVec1(2), testVec2(2),Scalar(1e-4));
}




/* Test Exponential and Logarithmic Map
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(EulerAnglesZyxSingleTest, testMaps){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;

  testVec = this->rotEulerAnglesZyxIdentity.logarithmicMap();
  ASSERT_NEAR(testVec(0), 0.0,1e-6);
  ASSERT_NEAR(testVec(1), 0.0,1e-6);
  ASSERT_NEAR(testVec(2), 0.0,1e-6);

  testVec = this->rotEulerAnglesZyxV3.logarithmicMap();
  EulerAnglesZyx rotExpMap = EulerAnglesZyx::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotEulerAnglesZyxV3.toImplementation(), rotExpMap.toImplementation(), Scalar(1e-4), "maps");

  testVec = this->rotEulerAnglesZyxV4.logarithmicMap();
  rotExpMap = EulerAnglesZyx::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotEulerAnglesZyxV4.toImplementation(), rotExpMap.toImplementation(), Scalar(1e-4), "maps");

  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rotExpMap = EulerAnglesZyx::exponentialMap(testVec);
  ASSERT_NEAR(rotExpMap.getDisparityAngle(this->rotEulerAnglesZyxIdentity),norm,1e-6);

  testVec.setZero();
  rotExpMap = EulerAnglesZyx::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotEulerAnglesZyxIdentity.toImplementation(), rotExpMap.toImplementation(), Scalar(1e-4), "maps");

}


/*  Test Box Operations
 * Assumes isNear() of Angle Axis is correct.
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(EulerAnglesZyxSingleTest, testBoxOperators){
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  EulerAnglesZyx rot;
  EulerAnglesZyx rot2;
  Vector testVec;

  // Test addition with 0
  testVec.setZero();
  rot = this->rotEulerAnglesZyxV3.boxPlus(testVec);
  ASSERT_EQ(rot.isNear(this->rotEulerAnglesZyxV3,1e-6),true);

  // Test subtraction of same elements
  testVec = this->rotEulerAnglesZyxV3.boxMinus(this->rotEulerAnglesZyxV3);
  ASSERT_NEAR(testVec(0),0.0,1e-6);
  ASSERT_NEAR(testVec(1),0.0,1e-6);
  ASSERT_NEAR(testVec(2),0.0,1e-6);

  // Test backward-forward
  testVec = this->rotEulerAnglesZyxV3.boxMinus(this->rotEulerAnglesZyxV4);
  rot = this->rotEulerAnglesZyxV4.boxPlus(testVec);
  ASSERT_EQ(rot.isNear(this->rotEulerAnglesZyxV3,1e-6),true);

  // Test forward-backward
  testVec = this->vec;
  rot = this->rotEulerAnglesZyxV3.boxPlus(testVec);
  testVec = rot.boxMinus(this->rotEulerAnglesZyxV3);
  ASSERT_NEAR(testVec(0),this->vec(0),Scalar(1e-4));
  ASSERT_NEAR(testVec(1),this->vec(1),Scalar(1e-4));
  ASSERT_NEAR(testVec(2),this->vec(2),Scalar(1e-4));

  // Test overlap with disparity angle
  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rot = this->rotEulerAnglesZyxV3.boxPlus(testVec);
  ASSERT_NEAR(rot.getDisparityAngle(this->rotEulerAnglesZyxV3),norm,Scalar(1e-4)); // Check distance between both
  rot2 = this->rotEulerAnglesZyxV3.boxPlus(2*testVec);
  ASSERT_NEAR(rot.getDisparityAngle(rot2),norm,Scalar(1e-4)); // Check distance to double
  rot2 = this->rotEulerAnglesZyxV3.boxPlus(-testVec);
  ASSERT_NEAR(rot.getDisparityAngle(rot2),2*norm,Scalar(1e-4)); // Check distance to reverse
}


// --------------------------------------------------------------------------------------------------- //
// -------- Testing for casting between different type of rotations and rotation Quaternions --------- //
// --------------------------------------------------------------------------------------------------- //

// Test conversion between rotation quaternion and rotation vectors
TYPED_TEST(EulerAnglesZyxRotationQuaternionPairTest, testConversionRotationQuaternionEulerAnglesZyx){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  RotationQuaternion rotQuat;
  EulerAnglesZyx rotEulerAnglesZyx;

  // TODO: add generic

  rotQuat = this->rotEulerAnglesZyxIdentity;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-3);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-3);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-3);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-3);
  rotQuat = this->rotEulerAnglesZyxQuarterX;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-3);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-3);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-3);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-3);
  rotQuat = this->rotEulerAnglesZyxQuarterY;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-3);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-3);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-3);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-3);
  rotQuat = this->rotEulerAnglesZyxQuarterZ;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-3);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-3);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-3);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-3);

  rotEulerAnglesZyx = this->rotQuatIdentity;
  ASSERT_NEAR(rotEulerAnglesZyx.x(), this->rotEulerAnglesZyxIdentity.x(),1e-3);
  ASSERT_NEAR(rotEulerAnglesZyx.y(), this->rotEulerAnglesZyxIdentity.y(),1e-3);
  ASSERT_NEAR(rotEulerAnglesZyx.z(), this->rotEulerAnglesZyxIdentity.z(),1e-3);
  rotEulerAnglesZyx = this->rotQuatQuarterX;
  ASSERT_NEAR(rotEulerAnglesZyx.x(), this->rotEulerAnglesZyxQuarterX.x(),1e-3);
  ASSERT_NEAR(rotEulerAnglesZyx.y(), this->rotEulerAnglesZyxQuarterX.y(),1e-3);
  ASSERT_NEAR(rotEulerAnglesZyx.z(), this->rotEulerAnglesZyxQuarterX.z(),1e-3);
  rotEulerAnglesZyx = this->rotQuatQuarterY;
  ASSERT_NEAR(rotEulerAnglesZyx.x(), this->rotEulerAnglesZyxQuarterY.x(),1e-3);
  ASSERT_NEAR(rotEulerAnglesZyx.y(), this->rotEulerAnglesZyxQuarterY.y(),1e-3);
  ASSERT_NEAR(rotEulerAnglesZyx.z(), this->rotEulerAnglesZyxQuarterY.z(),1e-3);
  rotEulerAnglesZyx = this->rotQuatQuarterZ;
  ASSERT_NEAR(rotEulerAnglesZyx.x(), this->rotEulerAnglesZyxQuarterZ.x(),1e-3);
  ASSERT_NEAR(rotEulerAnglesZyx.y(), this->rotEulerAnglesZyxQuarterZ.y(),1e-3);
  ASSERT_NEAR(rotEulerAnglesZyx.z(), this->rotEulerAnglesZyxQuarterZ.z(),1e-3);
}


// Test Inversion
TYPED_TEST(EulerAnglesZyxRotationQuaternionPairTest, testInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  RotationQuaternion rotQuat;
  EulerAnglesZyx rot1;
  EulerAnglesZyx rot2;

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

TYPED_TEST(EulerAnglesZyxSingleTest, testRotationOrder)
{
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;

  EulerAnglesZyx rot1(this->eigenVector3v1);
  EulerAnglesZyx rot2 = EulerAnglesZyx(this->eigenVector3v1(0),0,0)
                        *EulerAnglesZyx(0,this->eigenVector3v1(1),0)
                        *EulerAnglesZyx(0,0,this->eigenVector3v1(2));

  ASSERT_NEAR(rot1.x(), rot2.x(),1e-6);
  ASSERT_NEAR(rot1.y(), rot2.y(),1e-6);
  ASSERT_NEAR(rot1.z(), rot2.z(),1e-6);

}

TYPED_TEST(EulerAnglesZyxSingleTest, testRotationMatrix)
{
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::EulerAnglesZyx EulerAnglesZyx;

  Scalar x = M_PI_4;
  Scalar y = 1.2;
  Scalar z = -0.8;

//  Scalar x = 0.0;
//  Scalar y = 0.0;
//  Scalar z = M_PI_2;
  kindr::RotationMatrix<Scalar> rotMatKindr(EulerAnglesZyx(z, y, x));

  using std::cos;
  using std::sin;
  Eigen::Matrix<Scalar, 3, 3> rotMat;

  rotMat <<  cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
              cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x),
              -sin(y),                        cos(y)*sin(x),                        cos(x)*cos(y);

  KINDR_ASSERT_DOUBLE_MX_EQ(rotMat, rotMatKindr.matrix(), Scalar(1.0e-3), "rotation matrix");
  std::cout << "kindr: " << rotMatKindr.matrix() << "/n ours: " << rotMat << std::endl;
}

