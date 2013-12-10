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

#include "kinder/common/gtest_eigen.hpp"
#include "kinder/quaternions/QuaternionEigen.hpp"
#include "kinder/rotations/RotationEigen.hpp"

namespace rot = kinder::rotations::eigen_implementation;
namespace quat = kinder::quaternions::eigen_implementation;

template <typename RotationImplementation>
struct RotationTest {
  typedef RotationImplementation Rotation;
  typedef typename RotationImplementation::Scalar Scalar;

  static constexpr kinder::rotations::RotationUsage Usage = RotationImplementation::Usage;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
  typedef Eigen::Matrix<Scalar, 3, 4> Matrix3x4;
  Scalar tol;
  Vector3 vecX, vecY, vecZ, vecGeneric;

  RotationImplementation rotDefaultConstructor;
  RotationImplementation identity = RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(1, 0, 0, 0).cast<Scalar>()));
  RotationImplementation halfX =    RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(0, 1, 0, 0).cast<Scalar>()));
  RotationImplementation halfY =    RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(0, 0, 1, 0).cast<Scalar>()));
  RotationImplementation halfZ =    RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(0, 0, 0, 1).cast<Scalar>()));
  RotationImplementation rotGeneric =    RotationImplementation(rot::RotationQuaternion<Scalar, Usage>(Eigen::Quaterniond(1, 0, 0, 0).cast<Scalar>()));


  RotationTest() : tol(1e-6),
      vecX(Vector3::UnitX()),
      vecY(Vector3::UnitY()),
      vecZ(Vector3::UnitZ()),
      vecGeneric(Vector3(2,10,-7))
  {}
};

template <typename RotationImplementationPair>
struct RotationPairsTest : public ::testing::Test  {

  typedef typename RotationImplementationPair::first_type RotationImplementationA;
  typedef typename RotationImplementationPair::second_type RotationImplementationB;

  RotationTest<RotationImplementationA> rotTestA;
  RotationTest<RotationImplementationB> rotTestB;
};

template <typename RotationImplementation>
struct RotationSingleTest : public ::testing::Test{
  typedef RotationImplementation Rotation;
  typedef typename RotationImplementation::Scalar Scalar;



};

template <typename RotationQuaternionImplementation>
struct RotationQuaternionSingleTest : public ::testing::Test{
  typedef RotationQuaternionImplementation RotationQuaternion;
  typedef typename RotationQuaternionImplementation::Scalar Scalar;
  typedef typename RotationQuaternionImplementation::Implementation EigenQuat;
  typedef typename quat::UnitQuaternion<Scalar> UnitQuaternion;

  const EigenQuat eigenQuat1 = EigenQuat(0.0,0.36,0.48,0.8);
  const EigenQuat eigenQuat2 = EigenQuat(-0.48,-0.6,0.0,0.64);

  const EigenQuat eigenQuat1Conj = EigenQuat(0.0,-0.36,-0.48,-0.8);
  const EigenQuat eigenQuatIdentity = EigenQuat(1.0,0.0,0.0,0.0);
  const UnitQuaternion quat1 = UnitQuaternion(eigenQuat1);
  const UnitQuaternion quat2 = UnitQuaternion(eigenQuat2);
  const UnitQuaternion quatIdentity = UnitQuaternion(eigenQuatIdentity);
  const RotationQuaternion rotQuat1 = RotationQuaternion(eigenQuat1);
  const RotationQuaternion rotQuat2 = RotationQuaternion(eigenQuat2);
  const RotationQuaternion rotQuatIdentity = RotationQuaternion(eigenQuatIdentity);
};

//template <typename QuaternionImplementation>
//struct QuaternionsSingleTest : public ::testing::Test{
//    typedef QuaternionImplementation Quaternion;
//    typedef typename Quaternion::Scalar QuaternionScalar;
//    typedef Eigen::Quaternion<QuaternionScalar> EigenQuat;
//
//    const EigenQuat eigenQuat1 = EigenQuat(1.0,2.0,3.0,4.0);
//    const EigenQuat eigenQuat2 = EigenQuat(1.23,4.56,7.89,0.12);
//    const EigenQuat eigenQuat1Conj = EigenQuat(1.0,-2.0,-3.0,-4.0);
//    const EigenQuat eigenQuatIdentity = EigenQuat(1.0,0.0,0.0,0.0);
//    const Quaternion quat1 = Quaternion(eigenQuat1);
//    const Quaternion quat2 = Quaternion(eigenQuat2);
//    const Quaternion quatIdentity = Quaternion(eigenQuatIdentity);
//    const QuaternionScalar norm1 = 5.477225575051661;
//    const QuaternionScalar norm2 = 9.196357974763703;
//};


typedef ::testing::Types<
    rot::AngleAxisPD,
    rot::AngleAxisPF,
    rot::RotationQuaternionPD,
    rot::RotationQuaternionPF
> Types;

typedef ::testing::Types<
    rot::RotationQuaternionPD,
    rot::RotationQuaternionPF,
    rot::RotationQuaternionAD,
    rot::RotationQuaternionAF
> RotationQuaternionTypes;

typedef ::testing::Types<
    std::pair<rot::AngleAxisPD, rot::RotationQuaternionPD>
> TypePairs;

TYPED_TEST_CASE(RotationSingleTest, Types);
TYPED_TEST_CASE(RotationQuaternionSingleTest, RotationQuaternionTypes);
TYPED_TEST_CASE(RotationPairsTest, TypePairs);

// --------------------------------------------------------------------------------------------------- //
// ------------------------------ Testing for Rotation Quaternions only ------------------------------ //
// --------------------------------------------------------------------------------------------------- //

// Test Rotation Quaternion Constructors and access operator (relies on casting to base)
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionConstructors){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rot;
  ASSERT_EQ(rot.toUnitQuaternion().w(), this->eigenQuatIdentity.w());
  ASSERT_EQ(rot.toUnitQuaternion().x(), this->eigenQuatIdentity.x());
  ASSERT_EQ(rot.toUnitQuaternion().y(), this->eigenQuatIdentity.y());
  ASSERT_EQ(rot.toUnitQuaternion().z(), this->eigenQuatIdentity.z());

  RotationQuaternion rot2(this->eigenQuat1.w(),this->eigenQuat1.x(),this->eigenQuat1.y(),this->eigenQuat1.z());
  ASSERT_NEAR(rot2.toUnitQuaternion().w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot2.toUnitQuaternion().x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot2.toUnitQuaternion().y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot2.toUnitQuaternion().z(), this->eigenQuat1.z(),1e-6);

  RotationQuaternion rot3(this->eigenQuat1);
  ASSERT_NEAR(rot3.toUnitQuaternion().w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot3.toUnitQuaternion().x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot3.toUnitQuaternion().y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot3.toUnitQuaternion().z(), this->eigenQuat1.z(),1e-6);

  RotationQuaternion rot4(this->quat1);
  ASSERT_NEAR(rot4.toUnitQuaternion().w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot4.toUnitQuaternion().x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot4.toUnitQuaternion().y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot4.toUnitQuaternion().z(), this->eigenQuat1.z(),1e-6);

  RotationQuaternion rot5(rot3);
  ASSERT_NEAR(rot5.toUnitQuaternion().w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot5.toUnitQuaternion().x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot5.toUnitQuaternion().y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot5.toUnitQuaternion().z(), this->eigenQuat1.z(),1e-6);
}

// Test Rotation Quaternion Assignment Operator
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionAssignment){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rot;

  rot = this->quat1;
  ASSERT_NEAR(rot.toUnitQuaternion().w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().z(), this->eigenQuat1.z(),1e-6);

  rot = this->rotQuat1;
  ASSERT_NEAR(rot.toUnitQuaternion().w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().z(), this->eigenQuat1.z(),1e-6);
}

// Test Rotation Quaternion Inversion
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionInversion){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rot = this->rotQuat1.inverted();
  ASSERT_NEAR(rot.toUnitQuaternion().w(), this->eigenQuat1Conj.w(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().x(), this->eigenQuat1Conj.x(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().y(), this->eigenQuat1Conj.y(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().z(), this->eigenQuat1Conj.z(),1e-6);

  RotationQuaternion rot2 = rot.invert();
  ASSERT_NEAR(rot.toUnitQuaternion().w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().z(), this->eigenQuat1.z(),1e-6);
  ASSERT_NEAR(rot2.toUnitQuaternion().w(), this->eigenQuat1.w(),1e-6);
  ASSERT_NEAR(rot2.toUnitQuaternion().x(), this->eigenQuat1.x(),1e-6);
  ASSERT_NEAR(rot2.toUnitQuaternion().y(), this->eigenQuat1.y(),1e-6);
  ASSERT_NEAR(rot2.toUnitQuaternion().z(), this->eigenQuat1.z(),1e-6);
}

// Test Rotation Quaternion Setters
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionSetters){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;

  RotationQuaternion rot(this->rotQuat1);
  rot.setIdentity();
  ASSERT_NEAR(rot.toUnitQuaternion().w(), this->eigenQuatIdentity.w(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().x(), this->eigenQuatIdentity.x(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().y(), this->eigenQuatIdentity.y(),1e-6);
  ASSERT_NEAR(rot.toUnitQuaternion().z(), this->eigenQuatIdentity.z(),1e-6);
}

// Test Rotation Quaternion comparison (equality)
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionComparison){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  // Check equality comparison
  rotQuat = this->rotQuat1;
  ASSERT_EQ(rotQuat==this->rotQuat1,true);
  ASSERT_EQ(rotQuat==this->rotQuat2,false);
}

// Test Rotation Quaternion Concatenation
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionConcatenation){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  // Check multiplication of two generic rotation quaternions and compare with eigen results
  rotQuat = this->rotQuat1*this->rotQuat2;
  typename RotationQuaternion::Implementation eigenQuat12 = this->eigenQuat1*this->eigenQuat2;
  ASSERT_NEAR(rotQuat.toUnitQuaternion().w(), eigenQuat12.w(),1e-6);
  ASSERT_NEAR(rotQuat.toUnitQuaternion().x(), eigenQuat12.x(),1e-6);
  ASSERT_NEAR(rotQuat.toUnitQuaternion().y(), eigenQuat12.y(),1e-6);
  ASSERT_NEAR(rotQuat.toUnitQuaternion().z(), eigenQuat12.z(),1e-6);

  // Check result of multiplication of a generic rotation quaternion with identity
  rotQuat = this->rotQuat1*this->rotQuatIdentity;
  ASSERT_EQ(rotQuat==this->rotQuat1,true);
  rotQuat = this->rotQuatIdentity*this->rotQuat1;
  ASSERT_EQ(rotQuat==this->rotQuat1,true);
}

// Test Rotation Quaternion Vector Rotation
TYPED_TEST(RotationQuaternionSingleTest, testRotationQuaternionVectorRotation){
  typedef typename TestFixture::RotationQuaternion RotationQuaternion;
  typedef typename TestFixture::Scalar Scalar;
  RotationQuaternion rotQuat;

  // Check multiplication of two generic rotation quaternions and compare with eigen results
  rotQuat = this->rotQuat1*this->rotQuat2;
  typename RotationQuaternion::Implementation eigenQuat12 = this->eigenQuat1*this->eigenQuat2;
  ASSERT_NEAR(rotQuat.toUnitQuaternion().w(), eigenQuat12.w(),1e-6);
  ASSERT_NEAR(rotQuat.toUnitQuaternion().x(), eigenQuat12.x(),1e-6);
  ASSERT_NEAR(rotQuat.toUnitQuaternion().y(), eigenQuat12.y(),1e-6);
  ASSERT_NEAR(rotQuat.toUnitQuaternion().z(), eigenQuat12.z(),1e-6);

  // Check result of multiplication of a generic rotation quaternion with identity
  rotQuat = this->rotQuat1*this->rotQuatIdentity;
  ASSERT_EQ(rotQuat==this->rotQuat1,true);
  rotQuat = this->rotQuatIdentity*this->rotQuat1;
  ASSERT_EQ(rotQuat==this->rotQuat1,true);
}

//TYPED_TEST(RotationSingleTest, testConstructor){
//
//}
//
//TYPED_TEST(RotationSingleTest, testRotateVector){
////  ASSERT_EQ(this->vecGeneric, this->rotDefaultConstructor.rotate(this->vecGeneric));
////    for(auto & r : {TestFixture::halfX, TestFixture::halfY, TestFixture::halfZ}){
////      ASSERT_EQ(this->identity, r*r) << r*r; // TODO ASSERT_NEAR
////      ASSERT_TRUE(rm::rotations::areNearlyEqual(this->identity, r*r, this->tol)); // TODO ASSERT_NEAR
////
////      ASSERT_EQ(this->vecX, this->identity.rotate(this->vecX));
////      ASSERT_EQ(this->vecY, this->identity.rotate(this->vecY));
////      ASSERT_EQ(this->vecZ, this->identity.rotate(this->vecZ));
////    }
//}



// --------------------------------------------------------------------------------------------------- //
// ------------------------------- Testing for casting between classes ------------------------------- //
// --------------------------------------------------------------------------------------------------- //


//// Test constructors
//TYPED_TEST(RotationSingleTest, testRotationConstructors){
//  typedef typename TestFixture::Rotation Rotation;
//  typedef typename TestFixture::Scalar Scalar;
//
//  Rotation rot;
//}
//
//TYPED_TEST(RotationPairsTest, testConversion){
//
//}

TEST (RotationImplementationTest, testRotationVector) {
  rot::RotationVectorAD rvec;
  rot::RotationQuaternionAD rquat(kinder::quaternions::eigen_implementation::QuaternionD(1,2,3,4).toUnitQuaternion());
  rvec = rquat;
  std::cout << "rvec: " << rvec << std::endl;
  Eigen::Vector3d vec(1,2,3);
  std::cout << "rvec rotate: " << rvec.rotate(vec)-rquat.rotate(vec) << std::endl;
  std::cout << rquat << " | " <<  rot::RotationQuaternionAD(rvec) << std::endl;
  rot::EulerAnglesXyzAD xyz1(rvec);


//  xyz1 = rvec;
//  xyz1 = Eigen::Vector3d(0,0,0);
  rot::EulerAnglesXyzAD xyz2(rquat);
  std::cout << xyz1 << " | " <<  xyz2 << std::endl;



}
