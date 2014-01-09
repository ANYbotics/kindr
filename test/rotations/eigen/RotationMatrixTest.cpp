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
class RotationMatrixSingleTest : public ::testing::Test{
 public:
  typedef Rotation_ RotationMatrix;
  typedef typename Rotation_::Scalar Scalar;
  typedef Eigen::Matrix<Scalar,3,3> Matrix3x3;
  typedef Eigen::Matrix<Scalar,3,1> Vector;
  typedef typename rot::RotationQuaternion<Scalar, Rotation_::Usage> RotationQuaternion;
  Matrix3x3 eigenMatrix3x3Identity;
  Matrix3x3 eigenMatrix3x3v1;
  Matrix3x3 eigenMatrix3x3v2;


  RotationMatrix rotGeneric1;
  RotationMatrix rotGeneric2;


 RotationMatrix rotRotationMatrixQuarterX;
 RotationMatrix rotRotationMatrixQuarterY;
 RotationMatrix rotRotationMatrixQuarterZ;
 RotationMatrix rotRotationMatrixIdentity;





//  const RotationMatrix angleAxisGeneric1Minus2Pi = RotationMatrix(0.2-2.0*M_PI, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));
//  const RotationMatrix angleAxisGeneric2 = RotationMatrix(0.6, 2.0/sqrt(4.0+9.0+16.0), 3.0/sqrt(4.0+9.0+16.0), 4.0/sqrt(4.0+9.0+16.0));

//  const RotationMatrix rotRotationMatrixQuarterX = RotationMatrix(M_PI/2.0, 1.0, 0.0, 0.0);
//  const RotationMatrix rotRotationMatrixQuarterY = RotationMatrix(M_PI/2.0, 0.0, 1.0, 0.0);
//  const RotationMatrix rotRotationMatrixQuarterZ = RotationMatrix(M_PI/2.0, 0.0, 0.0, 1.0);
//  const RotationMatrix rotRotationMatrixIdentity = RotationMatrix(0.0, 1.0, 0.0, 0.0);
  const Vector vec = Vector(0.3,-1.5,0.6);
  const Vector vecX = Vector(1.0,0.0,0.0);
  const Vector vecY = Vector(0.0,1.0,0.0);
  const Vector vecZ = Vector(0.0,0.0,1.0);

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



    if (Rotation_::Usage == kindr::rotations::RotationUsage::PASSIVE) {

      rotRotationMatrixQuarterX = RotationMatrix( 1.0,  0.0,  0.0,
                                                  0.0,  0.0,  1.0,
                                                  0.0, -1.0,  0.0); // psi=0, theta=0, phi=pi/2

      rotRotationMatrixQuarterY = RotationMatrix( 0.0,  0.0,  -1.0,
                                                  0.0,  1.0,  0.0,
                                                  1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

      rotRotationMatrixQuarterZ = RotationMatrix( 0.0,  1.0,  0.0,
                                                  -1.0,  0.0,  0.0,
                                                   0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0

      rotRotationMatrix1 = RotationMatrix(RotationQuaternion(0.0,0.36,0.48,0.8).inverted());
      rotRotationMatrix2 = RotationMatrix(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)).inverted());
    }
    else {

      rotRotationMatrixQuarterX = RotationMatrix( 1.0,  0.0,  0.0,
                                                  0.0,  0.0,  -1.0,
                                                  0.0, 1.0,  0.0); // psi=0, theta=0, phi=pi/2

      rotRotationMatrixQuarterY = RotationMatrix( 0.0,  0.0,  1.0,
                                                0.0,  1.0,  0.0,
                                               -1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

      rotRotationMatrixQuarterZ = RotationMatrix( 0.0,  -1.0,  0.0,
                                                1.0,  0.0,  0.0,
                                                0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0

      rotRotationMatrix1 = RotationMatrix(RotationQuaternion(0.0,0.36,0.48,0.8));
      rotRotationMatrix2 = RotationMatrix(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)));

    }






    rotRotationMatrixIdentity = RotationMatrix( 1.0,  0.0,  0.0,
                                              0.0,  1.0,  0.0,
                                              0.0,  0.0,  1.0);






  }
};




template <typename RotationPair_>
struct RotationMatrixRotationQuaternionPairTest : public ::testing::Test{
  typedef typename RotationPair_::first_type RotationQuaternion;
  typedef typename RotationQuaternion::Scalar RotationQuaternionScalar;
  typedef typename RotationPair_::second_type RotationMatrix;
  typedef typename RotationMatrix::Scalar RotationMatrixScalar;
  typedef Eigen::Matrix<RotationQuaternionScalar,3,1> Vector;

  const RotationQuaternionScalar rotQuatSmallNumber = kindr::common::NumTraits<RotationQuaternionScalar>::dummy_precision()/10.0;
  const RotationQuaternionScalar angleAxisSmallNumber = kindr::common::NumTraits<RotationMatrixScalar>::dummy_precision()/10.0;

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
    if (RotationMatrix::Usage == kindr::rotations::RotationUsage::PASSIVE) {

      rotRotationMatrixQuarterX = RotationMatrix( 1.0,  0.0,  0.0,
                                                  0.0,  0.0,  1.0,
                                                  0.0, -1.0,  0.0); // psi=0, theta=0, phi=pi/2

      rotRotationMatrixQuarterY = RotationMatrix( 0.0,  0.0,  -1.0,
                                                  0.0,  1.0,  0.0,
                                                  1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

      rotRotationMatrixQuarterZ = RotationMatrix( 0.0,  1.0,  0.0,
                                                  -1.0,  0.0,  0.0,
                                                   0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0
    }
    else {

      rotRotationMatrixQuarterX = RotationMatrix( 1.0,  0.0,  0.0,
                                                  0.0,  0.0,  -1.0,
                                                  0.0, 1.0,  0.0); // psi=0, theta=0, phi=pi/2

      rotRotationMatrixQuarterY = RotationMatrix( 0.0,  0.0,  1.0,
                                                0.0,  1.0,  0.0,
                                               -1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

      rotRotationMatrixQuarterZ = RotationMatrix( 0.0,  -1.0,  0.0,
                                                1.0,  0.0,  0.0,
                                                0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0

    }


    rotRotationMatrixIdentity = RotationMatrix( 1.0,  0.0,  0.0,
                                              0.0,  1.0,  0.0,
                                              0.0,  0.0,  1.0);

  }


};

template <typename ImplementationPairs_>
struct RotationMatrixActiveTest : public RotationMatrixRotationQuaternionPairTest<ImplementationPairs_>{

};

template <typename ImplementationPairs_>
struct RotationMatrixPassiveTest : public RotationMatrixRotationQuaternionPairTest<ImplementationPairs_>{
};


typedef ::testing::Types<
    rot::RotationMatrixPD,
    rot::RotationMatrixPF,
    rot::RotationMatrixAD,
    rot::RotationMatrixAF
> RotationMatrixTypes;

typedef ::testing::Types<
    std::pair<rot::RotationQuaternionAD, rot::RotationMatrixAD>,
    std::pair<rot::RotationQuaternionAF, rot::RotationMatrixAF>
> RotationMatrixActiveTypes;

typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPD, rot::RotationMatrixPD>,
    std::pair<rot::RotationQuaternionPF, rot::RotationMatrixPF>
> RotationMatrixPassiveTypes;



typedef ::testing::Types<
    std::pair<rot::RotationQuaternionPF, rot::RotationMatrixPF>,
    std::pair<rot::RotationQuaternionPF, rot::RotationMatrixPD>,
    std::pair<rot::RotationQuaternionPD, rot::RotationMatrixPF>,
    std::pair<rot::RotationQuaternionPD, rot::RotationMatrixPD>,
    std::pair<rot::RotationQuaternionAF, rot::RotationMatrixAF>,
    std::pair<rot::RotationQuaternionAF, rot::RotationMatrixAD>,
    std::pair<rot::RotationQuaternionAD, rot::RotationMatrixAF>,
    std::pair<rot::RotationQuaternionAD, rot::RotationMatrixAD>
> TypeQuaternionRotationMatrixPairs;

TYPED_TEST_CASE(RotationMatrixSingleTest, RotationMatrixTypes);
TYPED_TEST_CASE(RotationMatrixRotationQuaternionPairTest, TypeQuaternionRotationMatrixPairs);
TYPED_TEST_CASE(RotationMatrixActiveTest, RotationMatrixActiveTypes);
TYPED_TEST_CASE(RotationMatrixPassiveTest, RotationMatrixPassiveTypes);


// Testing constructors
TYPED_TEST(RotationMatrixSingleTest, testConstructors){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

//  RotationMatrix rot;
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3Identity, rot.matrix(), 1e-4, "constructor");
//
//
//  RotationMatrix rot2(this->eigenMatrix3x3v1(0,0), this->eigenMatrix3x3v1(0,1), this->eigenMatrix3x3v1(0,2),
//                      this->eigenMatrix3x3v1(1,0), this->eigenMatrix3x3v1(1,1), this->eigenMatrix3x3v1(1,2),
//                      this->eigenMatrix3x3v1(2,0), this->eigenMatrix3x3v1(2,1), this->eigenMatrix3x3v1(2,2));
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), 1e-4, "constructor");
//
//
//  RotationMatrix rot4(this->eigenMatrix3x3v1);
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot4.matrix(), 1e-4, "constructor");
//
//
//  RotationMatrix rot5(rot4);
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot5.matrix(), 1e-4, "constructor");

  RotationMatrix rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3Identity, rot.matrix(), 1e-4, "constructor");


  RotationMatrix rot2(this->eigenMatrix3x3v1(0,0), this->eigenMatrix3x3v1(0,1), this->eigenMatrix3x3v1(0,2),
                      this->eigenMatrix3x3v1(1,0), this->eigenMatrix3x3v1(1,1), this->eigenMatrix3x3v1(1,2),
                      this->eigenMatrix3x3v1(2,0), this->eigenMatrix3x3v1(2,1), this->eigenMatrix3x3v1(2,2));
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), 1e-4, "constructor");


  RotationMatrix rot4(this->eigenMatrix3x3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot4.matrix(), 1e-4, "constructor");


  RotationMatrix rot5(rot4);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot5.matrix(), 1e-4, "constructor");
}

TYPED_TEST(RotationMatrixSingleTest, testAssignmentOperator){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

//  RotationMatrix rot(this->eigenMatrix3x3v1);
//  RotationMatrix rot1 = rot;
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), 1e-4, "constructor");

  RotationMatrix rot(this->eigenMatrix3x3v1);
  RotationMatrix rot1 = rot;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), 1e-4, "constructor");

}

TYPED_TEST(RotationMatrixSingleTest, testParenthesisOperator) {
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

//  RotationMatrix rot(this->eigenMatrix3x3v1);
//  RotationMatrix rot1;
//  rot1(rot);
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), 1e-4, "constructor");

  RotationMatrix rot(this->eigenMatrix3x3v1);
  RotationMatrix rot1;
  rot1(rot);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), 1e-4, "constructor");
}

TYPED_TEST(RotationMatrixSingleTest, testGetters)
{
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

//  RotationMatrix rot(this->eigenMatrix3x3v1);
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot.matrix(), 1e-4, "matrix()");
//
//  RotationMatrix rot1(this->eigenMatrix3x3v1);
//  RotationMatrix rot2 = rot1.getUnique();
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), 1e-4, "unique");
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), 1e-4, "unique");
//
//  RotationMatrix rot3(this->eigenMatrix3x3v1);
//  ASSERT_EQ(this->eigenMatrix3x3v1.determinant(), rot3.determinant());
//
//  RotationMatrix rot4(this->eigenMatrix3x3v1);
//  RotationMatrix rot5 = rot4.transposed();
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot4.matrix(), 1e-4, "transposed");
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot5.matrix(), 1e-4, "transposed");
//
//  RotationMatrix rot6 = rot4.transpose();
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot4.matrix(), 1e-4, "transpose");
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot6.matrix(), 1e-4, "transpose");

  RotationMatrix rot(this->eigenMatrix3x3v1);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot.matrix(), 1e-4, "matrix()");

  RotationMatrix rot1(this->eigenMatrix3x3v1);
  RotationMatrix rot2 = rot1.getUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), 1e-4, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), 1e-4, "unique");

  RotationMatrix rot3(this->eigenMatrix3x3v1);
  ASSERT_EQ(this->eigenMatrix3x3v1.determinant(), rot3.determinant());

  RotationMatrix rot4(this->eigenMatrix3x3v1);
  RotationMatrix rot5 = rot4.transposed();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot4.matrix(), 1e-4, "transposed");
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot5.matrix(), 1e-4, "transposed");

  RotationMatrix rot6 = rot4.transpose();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot4.matrix(), 1e-4, "transpose");
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1.transpose(), rot6.matrix(), 1e-4, "transpose");
}

TYPED_TEST(RotationMatrixSingleTest, testSetters)
{
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;

//  RotationMatrix rot(this->eigenMatrix3x3v1);
//  rot.setIdentity();
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3Identity, rot.matrix(), 1e-4, "identity");
//
//
//  RotationMatrix rot1(this->eigenMatrix3x3v1);
//  RotationMatrix rot2 = rot1.setUnique();
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), 1e-4, "unique");
//  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), 1e-4, "unique");


  RotationMatrix rot(this->eigenMatrix3x3v1);
  rot.setIdentity();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3Identity, rot.matrix(), 1e-4, "identity");


  RotationMatrix rot1(this->eigenMatrix3x3v1);
  RotationMatrix rot2 = rot1.setUnique();
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot1.matrix(), 1e-4, "unique");
  KINDR_ASSERT_DOUBLE_MX_EQ(this->eigenMatrix3x3v1, rot2.matrix(), 1e-4, "unique");

  rot.setFromVectors(this->vec, this->vec);
  KINDR_ASSERT_DOUBLE_MX_EQ(rot.matrix(), this->rotRotationMatrixIdentity.matrix(), 1e-4, "setFromVectors");

  rot.setFromVectors(this->vecX, this->vecY);
  KINDR_ASSERT_DOUBLE_MX_EQ(rot.matrix(), this->rotRotationMatrixQuarterZ.matrix(), 1e-4, "setFromVectors");
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
  ASSERT_NEAR(this->rotRotationMatrix1.getDisparityAngle(this->rotRotationMatrixIdentity),fabs(acos(RotationQuaternion(this->rotRotationMatrix1).w())*2),1e-6);
  ASSERT_NEAR(this->rotRotationMatrix2.getDisparityAngle(this->rotRotationMatrix1),fabs(acos((RotationQuaternion(this->rotRotationMatrix1).inverted()*RotationQuaternion(this->rotRotationMatrix2)).getUnique().w())*2),1e-6);
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
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterY;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterZ;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

  // Check concatenation of 3 different quarters
  rotRotationMatrix = this->rotRotationMatrixQuarterX.inverted()*this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterX;
  if(RotationMatrix::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationMatrix.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterX.inverted()*this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterX;
  if(RotationMatrix::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationMatrix.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.inverted().matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

   rotRotationMatrix = this->rotRotationMatrixQuarterY.inverted()*this->rotRotationMatrixQuarterX*this->rotRotationMatrixQuarterY;
  if(RotationMatrix::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationMatrix.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.inverted().matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterY.inverted()*this->rotRotationMatrixQuarterZ*this->rotRotationMatrixQuarterY;
  if(RotationMatrix::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationMatrix.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterZ.inverted()*this->rotRotationMatrixQuarterX*this->rotRotationMatrixQuarterZ;
  if(RotationMatrix::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationMatrix.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

  rotRotationMatrix = this->rotRotationMatrixQuarterZ.inverted()*this->rotRotationMatrixQuarterY*this->rotRotationMatrixQuarterZ;
  if(RotationMatrix::Usage == kindr::rotations::RotationUsage::ACTIVE){
    rotRotationMatrix.invert();
  }
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.inverted().matrix(), rotRotationMatrix.matrix(), 1e-4, "concatenation");

}


// Test Vector Rotation
TYPED_TEST(RotationMatrixSingleTest, testVectorRotation){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  Vector testVec;
  Vector testVec1;
  Vector testVec2;

  int signSwitch = 2*(RotationMatrix::Usage == kindr::rotations::RotationUsage::ACTIVE)-1;

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



/* Test Exponential and Logarithmic Map
 * Assumes getDisparityAngle() of Angle Axis is correct.
 */
TYPED_TEST(RotationMatrixSingleTest, testMaps){
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::Scalar Scalar;
  typedef typename TestFixture::Vector Vector;
  RotationMatrix rotRotationMatrix;
  Vector testVec;

  testVec = this->rotRotationMatrixIdentity.getLogarithmicMap();
  ASSERT_NEAR(testVec(0), 0.0,1e-6);
  ASSERT_NEAR(testVec(1), 0.0,1e-6);
  ASSERT_NEAR(testVec(2), 0.0,1e-6);

  testVec = this->rotRotationMatrix1.getLogarithmicMap();
  rotRotationMatrix.setExponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrix1.matrix(), rotRotationMatrix.matrix(), 1e-4, "maps");

  testVec = this->rotRotationMatrix2.getLogarithmicMap();
  rotRotationMatrix.setExponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrix2.matrix(), rotRotationMatrix.matrix(), 1e-4, "maps");

  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rotRotationMatrix.setExponentialMap(testVec);
  ASSERT_NEAR(rotRotationMatrix.getDisparityAngle(this->rotRotationMatrixIdentity),norm,1e-6);

  testVec.setZero();
  rotRotationMatrix.setExponentialMap(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rotRotationMatrix.matrix(), 1e-4, "maps");

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

//  RotationMatrix rotRotationMatrix1 = RotationMatrix(RotationQuaternion(0.0,0.36,0.48,0.8));
//  RotationMatrix rotRotationMatrix2 = RotationMatrix(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)));
//  RotationMatrix rotRotationMatrix1 = this->rotRotationMatrix1.getUnique();
//  RotationMatrix rotRotationMatrix2 = this->rotRotationMatrix2.getUnique();
    RotationMatrix rotRotationMatrix1 = RotationMatrix(RotationQuaternion(0.0,0.36,0.48,0.8).getUnique());
    RotationMatrix rotRotationMatrix2 = RotationMatrix(RotationQuaternion(4.0/sqrt(30.0),3.0/sqrt(30.0),1.0/sqrt(30.0),2.0/sqrt(30.0)).getUnique());

  Vector testVec;

  // Test addition with 0
  testVec.setZero();
  rot1 = rotRotationMatrix1.boxPlus(testVec);
  ASSERT_EQ(rot1.isNear(rotRotationMatrix1,1e-6),true);

  // Test subtraction of same elements
  testVec = this->rotRotationMatrix1.boxMinus(rotRotationMatrix1);
  ASSERT_NEAR(testVec(0),0.0,1e-6);
  ASSERT_NEAR(testVec(1),0.0,1e-6);
  ASSERT_NEAR(testVec(2),0.0,1e-6);

  // Test backward-forward
  testVec = rotRotationMatrix1.boxMinus(rotRotationMatrix2);
  rot1 = rotRotationMatrix2.boxPlus(testVec);
  KINDR_ASSERT_DOUBLE_MX_EQ(rotRotationMatrix1.matrix(), rot1.matrix(), 1e-4, "testBoxOperators");

//  ASSERT_EQ(rot1.isNear(this->rotRotationMatrix1,1e-4),true);

  // Test forward-backward
  testVec = this->vec;
  rot1 = rotRotationMatrix1.boxPlus(testVec);
  testVec = rot1.boxMinus(rotRotationMatrix1);
  ASSERT_NEAR(testVec(0),this->vec(0),1e-6) << "testVec: " << testVec.transpose() << " vec: " << this->vec.transpose();
  ASSERT_NEAR(testVec(1),this->vec(1),1e-6) << "testVec: " << testVec.transpose() << " vec: " << this->vec.transpose();
  ASSERT_NEAR(testVec(2),this->vec(2),1e-6) << "testVec: " << testVec.transpose() << " vec: " << this->vec.transpose();

  // Test overlap with disparity angle
  double norm = 0.1;
  testVec = this->vec/this->vec.norm()*norm;
  rot1 = rotRotationMatrix1.boxPlus(testVec);
  ASSERT_NEAR(rot1.getDisparityAngle(rotRotationMatrix1),norm,1e-4); // Check distance between both
  rot2 = this->rotRotationMatrix1.boxPlus(2*testVec);
  ASSERT_NEAR(rot1.getDisparityAngle(rot2),norm,1e-4); // Check distance to double
  rot2 = rotRotationMatrix1.boxPlus(-testVec);
  ASSERT_NEAR(rot1.getDisparityAngle(rot2),2*norm,1e-4); // Check distance to reverse
}



// --------------------------------------------------------------------------------------------------- //
// ---------------------------- RotationMatrixRotationQuaternionPairTest ---------------------------------- //
// --------------------------------------------------------------------------------------------------- //

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationQuaternionUsingConstructor){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;

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

  RotationMatrix rot1(this->rotQuatIdentity);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rot1.matrix(), 1e-4, "conversion");

  RotationMatrix rot2(this->rotQuatQuarterX);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.matrix(), rot2.matrix(), 1e-4, "conversion");

  RotationMatrix rot3(this->rotQuatQuarterY);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.matrix(), rot3.matrix(), 1e-4, "conversion");

  RotationMatrix rot4(this->rotQuatQuarterZ);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.matrix(), rot4.matrix(), 1e-4, "conversion");

}

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationQuaternionUsingAssignment) {
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
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
  RotationMatrix rot;

  rot = this->rotQuatIdentity;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rot.matrix(), 1e-4, "conversion");
  rot = this->rotQuatQuarterX;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.matrix(), rot.matrix(), 1e-4, "conversion");
  rot = this->rotQuatQuarterY;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.matrix(), rot.matrix(), 1e-4, "conversion");
  rot = this->rotQuatQuarterZ;
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.matrix(), rot.matrix(), 1e-4, "conversion");


}

TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testConvertToRotationQuaternionUsingParenthesisOperator){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
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
  RotationMatrix rot;

  rot(this->rotQuatIdentity);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixIdentity.matrix(), rot.matrix(), 1e-4, "conversion");
  rot(this->rotQuatQuarterX);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterX.matrix(), rot.matrix(), 1e-4, "conversion");
  rot(this->rotQuatQuarterY);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterY.matrix(), rot.matrix(), 1e-4, "conversion");
  rot(this->rotQuatQuarterZ);
  KINDR_ASSERT_DOUBLE_MX_EQ(this->rotRotationMatrixQuarterZ.matrix(), rot.matrix(), 1e-4, "conversion");

}

/* Test inversion
 * Assumes inversion of RotationQuaternion is correct.
 * Assumes conversion between RotationMatrix and RotationQuaternion is correct.
 */
TYPED_TEST(RotationMatrixRotationQuaternionPairTest, testInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
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
  KINDR_ASSERT_DOUBLE_MX_EQ(rot2.matrix(), rot1.matrix(), 1e-4, "inversion");

  rot3 = this->rotQuat1;
  rot5 = this->rotQuat1;
  rot4 = rot3.inverted();
  KINDR_ASSERT_DOUBLE_MX_EQ(rot4.matrix(), rot1.matrix(), 1e-4, "inversion");
  KINDR_ASSERT_DOUBLE_MX_EQ(rot3.matrix(), rot5.matrix(), 1e-4, "inversion");
}

/* Test getPassive()
 *  Assumes getPassive() of RotationQuaternion is correct.
 *  Assumes conversion between RotationMatrix and RotationQuaternion is correct.
 *  Assumes isNear() of RotationQuaternion is correct.
 */
TYPED_TEST(RotationMatrixActiveTest, testGetPassive){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::RotationQuaternionScalar RotationQuaternionScalar;
  typedef typename TestFixture::RotationMatrixScalar RotationMatrixScalar;

  rot::RotationMatrix<RotationMatrixScalar, kindr::rotations::RotationUsage::PASSIVE> rotRotationMatrixPassive;
  rot::RotationQuaternion<RotationQuaternionScalar, kindr::rotations::RotationUsage::PASSIVE> rotQuatPassive;

  rotRotationMatrixPassive = this->rotRotationMatrixIdentity.getPassive();
  rotQuatPassive = this->rotQuatIdentity.getPassive();
  ASSERT_EQ(true, rotQuatPassive.isNear(rotRotationMatrixPassive,1e-6));

  rotRotationMatrixPassive = this->rotRotationMatrixQuarterX.getPassive();
  rotQuatPassive = this->rotQuatQuarterX.getPassive();
  ASSERT_EQ(true, rotQuatPassive.isNear(rotRotationMatrixPassive,1e-6));

  rotRotationMatrixPassive = this->rotRotationMatrixQuarterY.getPassive();
  rotQuatPassive = this->rotQuatQuarterY.getPassive();
  ASSERT_EQ(true, rotQuatPassive.isNear(rotRotationMatrixPassive,1e-6));

  rotRotationMatrixPassive = this->rotRotationMatrixQuarterZ.getPassive();
  rotQuatPassive = this->rotQuatQuarterZ.getPassive();
  ASSERT_EQ(true, rotQuatPassive.isNear(rotRotationMatrixPassive,1e-6));

}
/* Test getActive()
 *  Assumes getActive() of RotationQuaternion is correct.
 *  Assumes conversion between RotationMatrix and RotationQuaternion is correct.
 *  Assumes isNear() of RotationQuaternion is correct.
 */
TYPED_TEST(RotationMatrixPassiveTest, testGetActive){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::RotationMatrix RotationMatrix;
  typedef typename TestFixture::RotationQuaternionScalar RotationQuaternionScalar;
  typedef typename TestFixture::RotationMatrixScalar RotationMatrixScalar;

  rot::RotationMatrix<RotationMatrixScalar, kindr::rotations::RotationUsage::ACTIVE> rotRotationMatrixActive;
  rot::RotationQuaternion<RotationQuaternionScalar, kindr::rotations::RotationUsage::ACTIVE> rotQuatActive;

  rotRotationMatrixActive = this->rotRotationMatrixIdentity.getActive();
  rotQuatActive = this->rotQuatIdentity.getActive();
  ASSERT_EQ(true, rotQuatActive.isNear(rotRotationMatrixActive,1e-6)) << "angle: " << rotQuatActive.getDisparityAngle(rotRotationMatrixActive);

  rotRotationMatrixActive = this->rotRotationMatrixQuarterX.getActive();
  rotQuatActive = this->rotQuatQuarterX.getActive();
  ASSERT_EQ(true, rotQuatActive.isNear(rotRotationMatrixActive,1e-6)) << "angle: " << rotQuatActive.getDisparityAngle(rotRotationMatrixActive);

  rotRotationMatrixActive = this->rotRotationMatrixQuarterY.getActive();
  rotQuatActive = this->rotQuatQuarterY.getActive();
  ASSERT_EQ(true, rotQuatActive.isNear(rotRotationMatrixActive,1e-6)) << "angle: " << rotQuatActive.getDisparityAngle(rotRotationMatrixActive);

  rotRotationMatrixActive = this->rotRotationMatrixQuarterZ.getActive();
  rotQuatActive = this->rotQuatQuarterZ.getActive();
  ASSERT_EQ(true, rotQuatActive.isNear(rotRotationMatrixActive,1e-6)) << "angle: " << rotQuatActive.getDisparityAngle(rotRotationMatrixActive);

}


