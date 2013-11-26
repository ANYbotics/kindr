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
#include "kinder/rotations/RotationEigen.hpp"

namespace rot = rm::rotations::eigen_implementation;
namespace quat = rm::quaternions::eigen_implementation;

template <typename RotationImplementation>
struct RotationTest {
  typedef RotationImplementation Rotation;
  typedef typename RotationImplementation::Scalar Scalar;

  static constexpr rm::rotations::RotationUsage Usage = RotationImplementation::Usage;
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
struct RotationSingleTest : public RotationTest<RotationImplementation>, public ::testing::Test  {

};


typedef ::testing::Types<
    rot::AngleAxisPD,
    rot::AngleAxisPF,
    rot::RotationQuaternionPD,
    rot::RotationQuaternionPF
> Types;

typedef ::testing::Types<
    std::pair<rot::AngleAxisPD, rot::RotationQuaternionPD>
> TypePairs;

TYPED_TEST_CASE(RotationSingleTest, Types);
TYPED_TEST_CASE(RotationPairsTest, TypePairs);


TYPED_TEST(RotationSingleTest, testConstructor){

}

TYPED_TEST(RotationSingleTest, testRotateVector){
  ASSERT_EQ(this->vecGeneric, this->rotDefaultConstructor.rotate(this->vecGeneric));
//	  for(auto & r : {TestFixture::halfX, TestFixture::halfY, TestFixture::halfZ}){
//	    ASSERT_EQ(this->identity, r*r) << r*r; // TODO ASSERT_NEAR
//		  ASSERT_TRUE(rm::rotations::areNearlyEqual(this->identity, r*r, this->tol)); // TODO ASSERT_NEAR
//
//	    ASSERT_EQ(this->vecX, this->identity.rotate(this->vecX));
//	    ASSERT_EQ(this->vecY, this->identity.rotate(this->vecY));
//	    ASSERT_EQ(this->vecZ, this->identity.rotate(this->vecZ));
//	  }
}

TYPED_TEST(RotationPairsTest, testConversion){




}
