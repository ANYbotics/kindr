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
class RotationMatrixSingleTest : public ::testing::Test{
 public:
  typedef Rotation_ RotationMatrix;
  typedef typename Rotation_::Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,3> Matrix3x3;
  typedef Eigen::Matrix<Scalar,3,1> Vector;
  typedef typename rot::RotationQuaternion<Scalar> RotationQuaternion;
  Matrix3x3 eigenMatrix3x3Identity;
  Matrix3x3 eigenMatrix3x3v1;
  Matrix3x3 eigenMatrix3x3v2;

  RotationMatrix rotGeneric1;
  RotationMatrix rotGeneric2;

  RotationMatrix rotRotationMatrixQuarterX;
  RotationMatrix rotRotationMatrixQuarterY;
  RotationMatrix rotRotationMatrixQuarterZ;
  RotationMatrix rotRotationMatrixIdentity;

  const Vector vec = Vector(0.3,-1.5,0.6);
  const Vector vecX = Vector(1.0,0.0,0.0);
  const Vector vecY = Vector(0.0,1.0,0.0);
  const Vector vecZ = Vector(0.0,0.0,1.0);
  const Vector vecXSmallError = Vector(1.0000001,0.0,0.0);

  RotationMatrix rotRotationMatrix1;
  RotationMatrix rotRotationMatrix2;

  RotationMatrixSingleTest() {
    eigenMatrix3x3Identity = Matrix3x3::Identity();
    eigenMatrix3x3v1 <<  879.923176281257e-003,    372.025551942260e-003,   -295.520206661340e-003,
                         -327.579672728226e-003,    925.564159446682e-003,    189.796060978687e-003,
                          344.131896020075e-003,   -70.1995402393384e-003,    936.293363584199e-003; //psi=0.4, theta=0.3 phi=0.2
    eigenMatrix3x3v2 <<  707.106781186548e-003,    0.00000000000000e+000,   -707.106781186547e-003,
                           0.00000000000000e+000,    1.00000000000000e+000,    0.00000000000000e+000,
                           707.106781186547e-003,    0.00000000000000e+000,    707.106781186548e-003; // psi=0, theta=pi/2, phi=0

    rotGeneric1 = RotationMatrix(eigenMatrix3x3v1);
    rotGeneric2 = RotationMatrix(eigenMatrix3x3v2);


    rotRotationMatrixQuarterX = RotationMatrix( 1.0,  0.0,  0.0,
                                                0.0,  0.0, -1.0,
                                                0.0,  1.0,  0.0); // psi=0, theta=0, phi=pi/2

    rotRotationMatrixQuarterY = RotationMatrix( 0.0,  0.0,  1.0,
                                                0.0,  1.0,  0.0,
                                               -1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

    rotRotationMatrixQuarterZ = RotationMatrix( 0.0, -1.0,  0.0,
                                                1.0,  0.0,  0.0,
                                                0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0

    rotRotationMatrixIdentity = RotationMatrix( 1.0,  0.0,  0.0,
                                                0.0,  1.0,  0.0,
                                                0.0,  0.0,  1.0);
  }

  virtual ~RotationMatrixSingleTest(){}
};




template <typename RotationPair_>
struct RotationMatrixRotationQuaternionPairTest : public ::testing::Test{
  typedef typename RotationPair_::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationQuaternion::Scalar Scalar;
  typedef typename RotationPair_::second_type RotationMatrix;
  typedef typename RotationMatrix::Scalar RotationMatrixScalar;
  typedef Eigen::Matrix<RotationQuaternionScalar,3,1> Vector;

  const RotationQuaternionScalar rotQuatSmallNumber = kindr::internal::NumTraits<RotationQuaternionScalar>::dummy_precision()/10.0;
  const RotationQuaternionScalar angleAxisSmallNumber = kindr::internal::NumTraits<RotationMatrixScalar>::dummy_precision()/10.0;

  const RotationQuaternion rotQuatQuarterX = RotationQuaternion(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const RotationQuaternion rotQuatQuarterY = RotationQuaternion(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const RotationQuaternion rotQuatQuarterZ = RotationQuaternion(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(1.0,0.0,0.0,0.0);
  const RotationQuaternion rotQuat1 = RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0));
  const RotationQuaternion rotQuat1Conj = RotationQuaternion(4.0/sqrt(30.0),-3.0/sqrt(30.0),-1.0/sqrt(30.0),-2.0/sqrt(30.0));

  RotationMatrix rotRotationMatrixQuarterX;
  RotationMatrix rotRotationMatrixQuarterY;
  RotationMatrix rotRotationMatrixQuarterZ;
  RotationMatrix rotRotationMatrixIdentity;

  RotationMatrixRotationQuaternionPairTest() {
    rotRotationMatrixQuarterX = RotationMatrix( 1.0,  0.0,  0.0,
                                                0.0,  0.0, -1.0,
                                                0.0,  1.0,  0.0); // psi=0, theta=0, phi=pi/2

    rotRotationMatrixQuarterY = RotationMatrix( 0.0,  0.0,  1.0,
                                                0.0,  1.0,  0.0,
                                               -1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

    rotRotationMatrixQuarterZ = RotationMatrix( 0.0, -1.0,  0.0,
                                                1.0,  0.0,  0.0,
                                                0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0

    rotRotationMatrixIdentity = RotationMatrix( 1.0,  0.0,  0.0,
                                                0.0,  1.0,  0.0,
                                                0.0,  0.0,  1.0);
  }
};

template <typename ImplementationPairs_>
struct RotationMatrixActiveTest : public RotationMatrixRotationQuaternionPairTest<ImplementationPairs_>{ // TODO: clean up

};

template <typename ImplementationPairs_>
struct RotationMatrixPassiveTest : public RotationMatrixRotationQuaternionPairTest<ImplementationPairs_>{
};


typedef ::testing::Types<
    rot::RotationMatrixPD,
    rot::RotationMatrixPF
> RotationMatrixTypes;

typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPD, rot::RotationMatrixPD>,
    std::pair<rot::RotationQuaternionPF, rot::RotationMatrixPF>
> RotationMatrixPassiveTypes;



typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPF, rot::RotationMatrixPF>,
    std::pair<rot::RotationQuaternionPF, rot::RotationMatrixPD>,
    std::pair<rot::RotationQuaternionPD, rot::RotationMatrixPF>,
    std::pair<rot::RotationQuaternionPD, rot::RotationMatrixPD>
> TypeQuaternionRotationMatrixPairs;

TYPED_TEST_CASE(RotationMatrixSingleTest, RotationMatrixTypes);
TYPED_TEST_CASE(RotationMatrixRotationQuaternionPairTest, TypeQuaternionRotationMatrixPairs);
TYPED_TEST_CASE(RotationMatrixPassiveTest, RotationMatrixPassiveTypes);

// Testing constructors
TYPED_TEST(RotationMatrixSingleTest, testConstructors){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

  RotationMatrix rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3Identity, rot.matrix(), Scalar(1e-4), "constructor");

  RotationMatrix rot2(this->eigenMatrix3x3v1(0,0), this->eigenMatrix3x3v1(0,1), this->eigenMatrix3x3v1(0,2),
                      this->eigenMatrix3x3v1(1,0), this->eigenMatrix3x3v1(1,1), this->eigenMatrix3x3v1(1,2),
                      this->eigenMatrix3x3v1(2,0), this->eigenMatrix3x3v1(2,1), this->eigenMatrix3x3v1(2,2));
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), Scalar(1e-4), "constructor");

  RotationMatrix rot4(this->eigenMatrix3x3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot4.matrix(), Scalar(1e-4), "constructor");

  RotationMatrix rot5(rot4);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot5.matrix(), Scalar(1e-4), "constructor");
}

TYPED_TEST(RotationMatrixSingleTest, testAssignmentOperator){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

  RotationMatrix rot(this->eigenMatrix3x3v1);
  RotationMatrix rot1 = rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), Scalar(1e-4), "constructor");
}

TYPED_TEST(RotationMatrixSingleTest, testParenthesisOperator) {
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

  RotationMatrix rot(this->eigenMatrix3x3v1);
  RotationMatrix rot1;
  rot1(rot);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), Scalar(1e-4), "constructor");
}

TYPED_TEST(RotationMatrixSingleTest, testGetters)
{
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

  RotationMatrix rot(this->eigenMatrix3x3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot.matrix(), Scalar(1e-4), "matrix()");

  RotationMatrix rot1(this->eigenMatrix3x3v1);
  RotationMatrix rot2 = rot1.getUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), Scalar(1e-4), "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), Scalar(1e-4), "unique");

  RotationMatrix rot3(this->eigenMatrix3x3v1);
  ASSERT_EQ(this->eigenMatrix3x3v1.determinant(), rot3.determinant());

  RotationMatrix rot4(this->eigenMatrix3x3v1);
  RotationMatrix rot5 = rot4.transposed();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot4.matrix(), Scalar(1e-4), "transposed");
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot5.matrix(), Scalar(1e-4), "transposed");

  RotationMatrix rot6 = rot4.transpose();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot4.matrix(), Scalar(1e-4), "transpose");
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot6.matrix(), Scalar(1e-4), "transpose");
}

TYPED_TEST(RotationMatrixSingleTest, testSetters)
{
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

  RotationMatrix rot(this->eigenMatrix3x3v1);
  rot.setIdentity();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3Identity, rot.matrix(), Scalar(1e-4), "identity");

  RotationMatrix rot1(this->eigenMatrix3x3v1);
  RotationMatrix rot2 = rot1.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), Scalar(1e-4), "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), Scalar(1e-4), "unique");

  rot.setFromVectors(this->vec, this->vec);
  KINDR_ASSERT_DOUBLE_MX_EQ(rot.matrix(), this->rotRotationMatrixIdentity.matrix(), Scalar(1e-4), "setFromVectors");

  rot.setFromVectors(this->vecX, this->vecY);
  KINDR_ASSERT_DOUBLE_MX_EQ(rot.matrix(), this->rotRotationMatrixQuarterZ.matrix(), Scalar(1e-4), "setFromVectors");

  rot.setFromVectors(Eigen::Matrix<Scalar, 3, 1>(1.0, 1.0, 0.0), Eigen::Matrix<Scalar, 3, 1>(-1.0, 1.0, 0.0));
  KINDR_ASSERT_DOUBLE_MX_EQ(rot.matrix(), this->rotRotationMatrixQuarterZ.matrix(), Scalar(1e-4), "setFromVectors");

  Eigen::Matrix<Scalar, 3, 1> vector1(1.0, 1.0, 0.0);
  Eigen::Matrix<Scalar, 3, 1> vector2(-1.0, 1.0, 0.0);
  rot.setFromVectors(vector1, vector2);
  KINDR_ASSERT_DOUBLE_MX_EQ(vector2, rot.rotate(vector1), Scalar(1e-4), "setFromVectors");

  rot.setFromVectors(this->vecX, this->vecX);
  KINDR_ASSERT_DOUBLE_MX_EQ(rot.matrix(), this->rotRotationMatrixIdentity.matrix(), Scalar(1e-4), "setFromVectors");

  rot.setFromVectors(this->vecX, this->vecXSmallError);
  KINDR_ASSERT_DOUBLE_MX_EQ(rot.matrix(), this->rotRotationMatrixIdentity.matrix(), Scalar(1e-4), "setFromVectors");
}


/* Test comparison (equality)
 *
 */
TYPED_TEST(RotationMatrixSingleTest, testComparisonEqual){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationMatrix rot;

  // Check equality comparison
  rot = this->rotGeneric1;
  ASSERT_EQ(true, rot==this->rotGeneric1);
  ASSERT_EQ(false, rot==this->rotGeneric2);
}

/* Test comparison (inequality)
 *
 */
TYPED_TEST(RotationMatrixSingleTest, testComparisonNotEqual){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationMatrix rot;

  // Check inequality comparison
  rot = this->rotGeneric1;
  ASSERT_EQ(false, rot!=this->rotGeneric1);
  ASSERT_EQ(true, rot!=this->rotGeneric2);
}

/* Test  getDisparityAngle
 * Assumes conversion between RotationMatrix and RotationQuaternion is correct.
 */
TYPED_TEST(RotationMatrixSingleTest, testGetDisparityAngle){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  ASSERT_NEAR(this->rotRotationMatrix1.getDisparityAngle(this->rotRotationMatrix1),0.0,1e-6);
  ASSERT_NEAR(this->rotRotationMatrix2.getDisparityAngle(this->rotRotationMatrix2),0.0,1e-6);
  ASSERT_NEAR(this->rotRotationMatrixIdentity.getDisparityAngle(this->rotRotationMatrixIdentity),0.0,1e-6);
  ASSERT_NEAR(this->rotRotationMatrix2.getDisparityAngle(this->rotRotationMatrix1),this->rotRotationMatrix1.getDisparityAngle(this->rotRotationMatrix2),1e-6);
  ASSERT_NEAR(this->rotRotationMatrix1.getDisparityAngle(this->rotRotationMatrixIdentity),calcRotationQuatDisparityAngleToIdentity(RotationQuaternion(this->rotRotationMatrix1)),1e-6);
  ASSERT_NEAR(this->rotRotationMatrix2.getDisparityAngle(this->rotRotationMatrix1),calcRotationQuatDisparityAngle(RotationQuaternion(this->rotRotationMatrix1), RotationQuaternion(this->rotRotationMatrix2)),1e-6);
}


// Test isNear
TYPED_TEST(RotationMatrixSingleTest, testIsNear){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationMatrix rot;

  // Check isNear
  rot = this->rotGeneric1;
  ASSERT_EQ(rot.isNear(this->rotGeneric1,1e-6),true);
  ASSERT_EQ(rot.isNear(this->rotGeneric2,1e-6),false);
}



/* Test Concatenation
 * Assumes isNear is correct.
 */
TYPED_TEST(RotationMatrixSingleTest, testConcatenation){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationMatrix rotRotationMatrix;

  // Check result of multiplication of a generic rotation with identity
  rotRotationMatrix = this->rotRotationMatrix1*this->rotRotationMatrixIdentity;
  ASSERT_EQ(rotRotationMatrix.isNear(this->rotRotationMatrix1,1e-6),true);
  rotRotationMatrix = this->rotRotationMatrixIdentity*this->rotRotationMatrix1;
  ASSERT_EQ(rotRotationMatrix.isNear(this->rotRotationMatrix1,1e-6),true);

  // Check concatenation of 4 quarters
  rotRotationMatrix = this->rotRotationMatrixQuarterX*this->rotRotationMatrixQuarterX*this->rotRotationMatrixQuarterX*this->rotRotationMatrixQuarterX;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterY;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterZ;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");

  // Check concatenation of 3 different quarters
  rotRotationMatrix = this->rotRotationMatrixQuarterX.inverted()*this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterX;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.inverted().matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterX.inverted()*this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterX;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterY.inverted()*this->rotRotationMatrixQuarterX*this->rotRotationMatrixQuarterY;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterY.inverted()*this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterY;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.inverted().matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterZ.inverted()*this->rotRotationMatrixQuarterX*this->rotRotationMatrixQuarterZ;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.inverted().matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterZ.inverted()*this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterZ;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.matrix(), rotRotationMatrix.matrix(), Scalar(1e-4), "concatenation");
}


// Test fix
TYPED_TEST(RotationMatrixSingleTest, testFix){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationMatrix rot;
  RotationMatrix rotModified;

  // Check fix
  rot = this->rotRotationMatrix1;
  rotModified = rot;
  rotModified.toImplementation() *= 1.1;
  rotModified.fix();
  KINDR_ASSERT_DOUBLE_MX_EQ(rotModified.matrix(), rot.matrix(), Scalar(1e-4), "fix");
}


// Test Vector Rotation
TYPED_TEST(RotationMatrixSingleTest, testVectorRotation){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;
  Vector testVec1;
  Vector testVec2;

  int signSwitch = 1;

  // Check rotation of base vectors around main axis
  testVec = this->rotRotationMatrixQuarterX.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotRotationMatrixQuarterX.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotRotationMatrixQuarterX.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),1e-6);
  testVec = this->rotRotationMatrixQuarterY.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotRotationMatrixQuarterY.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotRotationMatrixQuarterY.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),1e-6);
  testVec = this->rotRotationMatrixQuarterZ.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),1e-6);
  testVec = this->rotRotationMatrixQuarterZ.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),1e-6);
  testVec = this->rotRotationMatrixQuarterZ.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  testVec = this->rotRotationMatrixQuarterX.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotRotationMatrixQuarterX.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotRotationMatrixQuarterX.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecY(2),1e-6);
  testVec = this->rotRotationMatrixQuarterY.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecZ(2),1e-6);
  testVec = this->rotRotationMatrixQuarterY.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotRotationMatrixQuarterY.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecX(2),1e-6);
  testVec = this->rotRotationMatrixQuarterZ.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), -signSwitch*this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), -signSwitch*this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), -signSwitch*this->vecY(2),1e-6);
  testVec = this->rotRotationMatrixQuarterZ.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), signSwitch*this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), signSwitch*this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), signSwitch*this->vecX(2),1e-6);
  testVec = this->rotRotationMatrixQuarterZ.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  // Check rotation with Identity
  testVec = this->rotRotationMatrixIdentity.rotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotRotationMatrixIdentity.rotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotRotationMatrixIdentity.rotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);
  testVec = this->rotRotationMatrixIdentity.inverseRotate(this->vecX);
  ASSERT_NEAR(testVec(0), this->vecX(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecX(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecX(2),1e-6);
  testVec = this->rotRotationMatrixIdentity.inverseRotate(this->vecY);
  ASSERT_NEAR(testVec(0), this->vecY(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecY(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecY(2),1e-6);
  testVec = this->rotRotationMatrixIdentity.inverseRotate(this->vecZ);
  ASSERT_NEAR(testVec(0), this->vecZ(0),1e-6);
  ASSERT_NEAR(testVec(1), this->vecZ(1),1e-6);
  ASSERT_NEAR(testVec(2), this->vecZ(2),1e-6);

  // Check combination between concatenation and rotate
  testVec1 = this->rotRotationMatrix2.rotate(this->rotRotationMatrix1.rotate(this->vec));
  testVec2 = (this->rotRotationMatrix2*this->rotRotationMatrix1).rotate(this->vec);
  ASSERT_NEAR(testVec1(0), testVec2(0),1e-6);
  ASSERT_NEAR(testVec1(1), testVec2(1),1e-6);
  ASSERT_NEAR(testVec1(2), testVec2(2),1e-6);
}




TYPED_TEST(RotationMatrixSingleTest, testExpMap){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  typedef typename TestFixture::Matrix3x3 Matrix;

  Vector vector(1.0, 0.0, 0.0);
  RotationMatrix rotMeas = RotationMatrix::exponentialMap(vector);

  RotationMatrix rotTrue;
  rotTrue.toImplementation() << 1.000000000000000e+00, 0, 0,
                      0,     5.403023058681397e-01,    -8.414709848078965e-01,
                      0,     8.414709848078965e-01,     5.403023058681398e-01;
  KINDR_ASSERT_DOUBLE_MX_EQ(rotMeas.matrix(), rotTrue.matrix(), Scalar(1e-4), "testExpMap");


  vector << 0.1, 0.2, 0.3;
  rotTrue.toImplementation() << 9.357548032779188e-01,    -2.831649605650737e-01,     2.101917059507429e-01,
     3.029327134026372e-01,     9.505806179060916e-01,    -6.803131640494003e-02,
    -1.805400766943977e-01,     1.273345749176302e-01,     9.752903089530458e-01;
  rotMeas = RotationMatrix::exponentialMap(vector);
  KINDR_ASSERT_DOUBLE_MX_EQ(rotMeas.toImplementation(), rotTrue.toImplementation(), Scalar(1e-4), "expMap");

}

TYPED_TEST(RotationMatrixSingleTest, testLogMap){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  typedef typename TestFixture::Matrix3x3 Matrix;

  RotationMatrix rot;
  rot.toImplementation() << 879.923176281257e-003,    372.025551942260e-003,   -295.520206661340e-003,
                            -327.579672728226e-003,    925.564159446682e-003,    189.796060978687e-003,
                             344.131896020075e-003,   -70.1995402393384e-003,    936.293363584199e-003; //psi=0.4, theta=0.3 phi=0.2

  Vector vectorCorrect(  -1.358983490410654e-01,
                         -3.343428285240700e-01,
                         -3.656800136918290e-01);
  Vector vector = rot.logarithmicMap();
  KINDR_ASSERT_DOUBLE_MX_EQ(vector, vectorCorrect, Scalar(1e-4), "logMap1");


  rot.toImplementation() << 8.799231762812570e-01,    -3.720255519422596e-01,     2.955202066613395e-01,
                            4.357321314618704e-01,     8.798380333042382e-01,    -1.897960609786874e-01,
                           -1.894009330885121e-01,     2.957736023606357e-01,     9.362933635841992e-01; //psi=-0.4, theta=-0.3 phi=-0.2

  vectorCorrect <<  2.558835877170818e-01,2.555418313115193e-01, 4.256689608943892e-01;
  vector = rot.logarithmicMap();
  KINDR_ASSERT_DOUBLE_MX_EQ(vector, vectorCorrect, Scalar(1e-4), "logMap2");

}



/* Test Exponential and Logarithmic Map
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(RotationMatrixSingleTest, testMaps){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;

  testVec = this->rotRotationMatrixIdentity.logarithmicMap();
  ASSERT_NEAR(testVec(0), 0.0,1e-6);
  ASSERT_NEAR(testVec(1), 0.0,1e-6);
  ASSERT_NEAR(testVec(2), 0.0,1e-6);

  testVec = this->rotRotationMatrix1.logarithmicMap();
  RotationMatrix rotRotationMatrixExpMap = RotationMatrix::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrix1.toImplementation(), rotRotationMatrixExpMap.toImplementation(), Scalar(1e-3), "logExpMapMat1");

  testVec = this->rotRotationMatrix2.logarithmicMap();
  rotRotationMatrixExpMap = RotationMatrix::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrix2.matrix(), rotRotationMatrixExpMap.matrix(), Scalar(1e-4), "logExpMapMat2");

  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rotRotationMatrixExpMap = RotationMatrix::exponentialMap(testVec);
  ASSERT_NEAR(rotRotationMatrixExpMap.getDisparityAngle(this->rotRotationMatrixIdentity),norm,Scalar(1e-6));

  testVec.setZero();
  rotRotationMatrixExpMap = RotationMatrix::exponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rotRotationMatrixExpMap.matrix(), Scalar(1e-4), "maps");

}

TYPED_TEST(RotationMatrixSingleTest, testBoxPlus){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  typedef typename TestFixture::Matrix3x3 Matrix;

  // Test groundtruth data
   RotationMatrix rotRotationMatrix3;

   rotRotationMatrix3.toImplementation() << 1.0, 0.0, 0.0,
                                     0.0, 0, 1.0,
                                     0.0, -1.0, 0;

   Vector vectorInput = 1e-5*Vector(1.0, 0.0, 0.0);
   RotationMatrix rotRotationMatrix4 = rotRotationMatrix3.boxPlus(vectorInput);
   RotationMatrix rotRotationMatrix4Correct;

   rotRotationMatrix4Correct.toImplementation() <<     1.000000000000000e+00,   0,     0,
                                                 0,     9.999999999833334e-06,     9.999999999500001e-01,
                                                 0,    -9.999999999500001e-01,     9.999999999833334e-06;
   KINDR_ASSERT_DOUBLE_MX_EQ(rotRotationMatrix4Correct.matrix(), rotRotationMatrix4.matrix(), Scalar(1e-4), "boxPlus");
}


TYPED_TEST(RotationMatrixSingleTest, testBoxMinus){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;

  RotationMatrix rotOne_v1;
  rotOne_v1.toImplementation() << 1.0, 0.0, 0.0,
                                  0.0, 0, 1.0,
                                  0.0, -1.0, 0;

  RotationMatrix rotTwo_v1;
  rotTwo_v1.toImplementation() << 1.000000000000000e+00,   0,     0,
                                  0,     9.999999999833334e-06,     9.999999999500001e-01,
                                  0,    -9.999999999500001e-01,     9.999999999833334e-06;
  Vector vectorTrue_v1(-9.999999999999996e-06, 0.0, 0.0);

  Vector vectorMeas_v1 = rotOne_v1.boxMinus(rotTwo_v1);
  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(vectorTrue_v1, vectorMeas_v1, 1e-5, 1e-5, "boxMinus");


  //Direct cosine matrix from Euler angles zyx: z=0.1, y=0.2 x=0.3
  rotOne_v1.toImplementation() <<       9.751703272018158e-01,     9.784339500725571e-02,    -1.986693307950612e-01,
                                      -3.695701352462509e-02,     9.564250858492324e-01,     2.896294776255156e-01,
                                       2.183506631463344e-01,    -2.750958473182437e-01,     9.362933635841992e-01;
  //Direct cosine matrix from Euler angles zyx: z=0.15, y=0.23 x=0.37
  rotTwo_v1.toImplementation() <<
      9.627331709395648e-01,     1.455028877219050e-01,    -2.279775235351884e-01,
     -5.781078120571753e-02,     9.341780226358523e-01,     3.520927940196359e-01,
      2.642021104174517e-01,    -3.257918533185810e-01,     9.077758055611578e-01;

  vectorTrue_v1 <<      5.931305743112836e-02,
      4.439329307381447e-02,
      3.627918600228441e-02;
  vectorMeas_v1 = rotOne_v1.boxMinus(rotTwo_v1);
  std::cout << "norms: " << vectorTrue_v1.norm() << ", " << vectorMeas_v1.norm() <<std::endl;
  KINDR_ASSERT_DOUBLE_MX_EQ_ABS_REL(vectorTrue_v1, vectorMeas_v1, 1e-2, 1e-2, "boxMinus");

}


/*  Test Box Operations
 * Assumes isNear() of Angle Axis is correct.
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(RotationMatrixSingleTest, testBoxOperators){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;

  RotationMatrix rot1;
  RotationMatrix rot2;

  this->rotRotationMatrix1 = RotationMatrix(RotationQuaternion(0.0,0.36,0.48,0.8).getUnique());
  this->rotRotationMatrix2 = RotationMatrix(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)).getUnique());

  Vector testVec;

  // Test addition with 0
  testVec.setZero();
  rot1 = this->rotRotationMatrix1.boxPlus(testVec);
  ASSERT_EQ(rot1.isNear(this->rotRotationMatrix1,1e-6),true);

  // Test subtraction of same elements
  testVec = this->rotRotationMatrix1.boxMinus(this->rotRotationMatrix1);
  ASSERT_NEAR(testVec(0),0.0,1e-6);
  ASSERT_NEAR(testVec(1),0.0,1e-6);
  ASSERT_NEAR(testVec(2),0.0,1e-6);

  // Test backward-forward
  testVec = this->rotRotationMatrix1.boxMinus(this->rotRotationMatrix2);
  rot1 = this->rotRotationMatrix2.boxPlus(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrix1.matrix(), rot1.matrix(), Scalar(1e-4), "testBoxOperators");

//  ASSERT_EQ(rot1.isNear(this->rotRotationMatrix1,Scalar(1e-4)),true);

  // Test forward-backward
  testVec = this->vec;
  rot1 = this->rotRotationMatrix1.boxPlus(testVec);
  testVec = rot1.boxMinus(this->rotRotationMatrix1);
  ASSERT_NEAR(testVec(0),this->vec(0),1e-6) << "testVec: " << testVec.transpose() << " vec: " << this->vec.transpose();
  ASSERT_NEAR(testVec(1),this->vec(1),1e-6) << "testVec: " << testVec.transpose() << " vec: " << this->vec.transpose();
  ASSERT_NEAR(testVec(2),this->vec(2),1e-6) << "testVec: " << testVec.transpose() << " vec: " << this->vec.transpose();

  // Test overlap with disparity angle
  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rot1 = this->rotRotationMatrix1.boxPlus(testVec);
  ASSERT_NEAR(rot1.getDisparityAngle(this->rotRotationMatrix1),norm,Scalar(1e-4)); // Check distance between both
  rot2 = this->rotRotationMatrix1.boxPlus(2*testVec);
  ASSERT_NEAR(rot1.getDisparityAngle(rot2),norm,Scalar(1e-4)); // Check distance to double
  rot2 = this->rotRotationMatrix1.boxPlus(-testVec);
  ASSERT_NEAR(rot1.getDisparityAngle(rot2),2*norm,Scalar(1e-4)); // Check distance to reverse


}



// --------------------------------------------------------------------------------------------------- //
// ---------------------------- RotationMatrixRotationQuaternionPairTest ---------------------------------- //
// --------------------------------------------------------------------------------------------------- //

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationQuaternionUsingConstructor){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rotQuat2(this->rotRotationMatrixIdentity);
  ASSERT_NEAR(rotQuat2.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat2.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat2.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat2.z(), this->rotQuatIdentity.z(),1e-6);
  RotationQuaternion rotQuat3(this->rotRotationMatrixQuarterX);
  ASSERT_NEAR(rotQuat3.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat3.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat3.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat3.z(), this->rotQuatQuarterX.z(),1e-6);
  RotationQuaternion rotQuat4(this->rotRotationMatrixQuarterY);
  ASSERT_NEAR(rotQuat4.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat4.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat4.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat4.z(), this->rotQuatQuarterY.z(),1e-6);
  RotationQuaternion rotQuat5(this->rotRotationMatrixQuarterZ);
  ASSERT_NEAR(rotQuat5.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat5.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat5.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat5.z(), this->rotQuatQuarterZ.z(),1e-6);
}

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationMatrixUsingConstructor){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

  RotationMatrix rot1(this->rotQuatIdentity);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rot1.matrix(), Scalar(1e-4), "conversion");

  RotationMatrix rot2(this->rotQuatQuarterX);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.matrix(), rot2.matrix(), Scalar(1e-4), "conversion");

  RotationMatrix rot3(this->rotQuatQuarterY);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.matrix(), rot3.matrix(), Scalar(1e-4), "conversion");

  RotationMatrix rot4(this->rotQuatQuarterZ);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.matrix(), rot4.matrix(), Scalar(1e-4), "conversion");

}

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationQuaternionUsingAssignment) {
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  // TODO: add generic

  rotQuat = this->rotRotationMatrixIdentity;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-6);
  rotQuat = this->rotRotationMatrixQuarterX;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-6);
  rotQuat = this->rotRotationMatrixQuarterY;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-6);
  rotQuat = this->rotRotationMatrixQuarterZ;
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-6);
}

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationMatrixUsingAssignment) {
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationMatrix rot;

  rot = this->rotQuatIdentity;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rot.matrix(), Scalar(1e-4), "conversion");
  rot = this->rotQuatQuarterX;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.matrix(), rot.matrix(), Scalar(1e-4), "conversion");
  rot = this->rotQuatQuarterY;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.matrix(), rot.matrix(), Scalar(1e-4), "conversion");
  rot = this->rotQuatQuarterZ;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.matrix(), rot.matrix(), Scalar(1e-4), "conversion");


}

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationQuaternionUsingParenthesisOperator){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  rotQuat(this->rotRotationMatrixIdentity);
  ASSERT_NEAR(rotQuat.w(), this->rotQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatIdentity.z(),1e-6);
  rotQuat(this->rotRotationMatrixQuarterX);
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterX.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterX.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterX.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterX.z(),1e-6);
  rotQuat(this->rotRotationMatrixQuarterY);
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterY.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterY.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterY.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterY.z(),1e-6);
  rotQuat(this->rotRotationMatrixQuarterZ);
  ASSERT_NEAR(rotQuat.w(), this->rotQuatQuarterZ.w(),1e-6);
  ASSERT_NEAR(rotQuat.x(), this->rotQuatQuarterZ.x(),1e-6);
  ASSERT_NEAR(rotQuat.y(), this->rotQuatQuarterZ.y(),1e-6);
  ASSERT_NEAR(rotQuat.z(), this->rotQuatQuarterZ.z(),1e-6);
}

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationMatrixUsingParenthesisOperator){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationMatrix rot;

  rot(this->rotQuatIdentity);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rot.matrix(), Scalar(1e-4), "conversion");
  rot(this->rotQuatQuarterX);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.matrix(), rot.matrix(), Scalar(1e-4), "conversion");
  rot(this->rotQuatQuarterY);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.matrix(), rot.matrix(), Scalar(1e-4), "conversion");
  rot(this->rotQuatQuarterZ);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.matrix(), rot.matrix(), Scalar(1e-4), "conversion");

}

/* Test inversion
 * Assumes inversion of RotationQuaternion is correct.
 * Assumes conversion between RotationMatrix and RotationQuaternion is correct.
 */
TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;
  RotationMatrix rot1;
  RotationMatrix rot2;
  RotationMatrix rot3;
  RotationMatrix rot4;
  RotationMatrix rot5;

  // Use rotation quaternion method as reference
  rotQuat = this->rotQuat1.inverted();
  rot1 = rotQuat;

  // Use rotation vector method and compare
  rot2 = this->rotQuat1;
  rot2.invert();
  KINDR_ASSERT_DOUBLE_MX_EQ(rot2.matrix(), rot1.matrix(), Scalar(1e-4), "inversion");

  rot3 = this->rotQuat1;
  rot5 = this->rotQuat1;
  rot4 = rot3.inverted();
  KINDR_ASSERT_DOUBLE_MX_EQ(rot4.matrix(), rot1.matrix(), Scalar(1e-4), "inversion");
  KINDR_ASSERT_DOUBLE_MX_EQ(rot3.matrix(), rot5.matrix(), Scalar(1e-4), "inversion");
}



