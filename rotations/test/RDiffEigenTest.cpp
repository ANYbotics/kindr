/*
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

#include <Eigen/Core>

#include <gtest/gtest.h>

#include "kindr/rotations/RDiffEigen.hpp"
#include "kindr/common/gtest_eigen.hpp"



namespace rot = kindr::rotations::eigen_impl;

template <typename AngularVelocityImplementation>
struct AngularVelocityTest: public ::testing::Test {
  typedef AngularVelocityImplementation AngularVelocity;
  typedef typename AngularVelocity::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

  Scalar tol;
  Vector3 vecZero, vec1, vec2, vecAdd, vecSubtract;

  AngularVelocity velDefault;
  AngularVelocity velFromThreeValues;
  AngularVelocity velFromEigen;
  AngularVelocity vel2FromEigen;
  AngularVelocity velFromVel;

  AngularVelocityTest() : tol(1e-6),
      vecZero(Vector3::Zero()),
      vec1(10,20,30),
      vec2(1,2,3),
      vecAdd(11,22,33),
      vecSubtract(9,18,27),
      velFromThreeValues(vec1.x(), vec1.y(), vec1.z()),
      velFromEigen(vec1),
      vel2FromEigen(vec2),
      velFromVel(velFromEigen)
  {}
};


typedef ::testing::Types<
    rot::AngularVelocityAD,
    rot::AngularVelocityAF
> Types;


TYPED_TEST_CASE(AngularVelocityTest, Types);

TYPED_TEST(AngularVelocityTest, testAngularVelocity)
{
  typedef typename TestFixture::AngularVelocity AngularVelocity;

   // default constructor
   ASSERT_EQ(this->velDefault.x(), this->vecZero.x()) << "Default constructor needs to initialize x-component to zero!";
   ASSERT_EQ(this->velDefault.y(), this->vecZero.y()) << "Default constructor needs to initialize y-component to zero!";
   ASSERT_EQ(this->velDefault.z(), this->vecZero.z()) << "Default constructor needs to initialize z-component to zero!";

   // constructor with three values (x,y,z)
   ASSERT_EQ(this->velFromThreeValues.x(), this->vec1.x()) << "Three-Value Constructor needs to first initialize x-component!";
   ASSERT_EQ(this->velFromThreeValues.y(), this->vec1.y()) << "Three-Value Constructor needs to second initialize y-component!";
   ASSERT_EQ(this->velFromThreeValues.z(), this->vec1.z()) << "Three-Value Constructor needs to third initialize z-component!";

   // constructor with Eigen vector
   ASSERT_EQ(this->velFromEigen.x(), this->vec1.x()) << "Base Constructor needs to first initialize x-component!";
   ASSERT_EQ(this->velFromEigen.y(), this->vec1.y()) << "Base Constructor needs to second initialize y-component!";
   ASSERT_EQ(this->velFromEigen.z(), this->vec1.z()) << "Base Constructor needs to third initialize z-component!";

   // constructor with Position3
   ASSERT_EQ(this->velFromVel.x(), this->vec1.x());
   ASSERT_EQ(this->velFromVel.y(), this->vec1.y());
   ASSERT_EQ(this->velFromVel.z(), this->vec1.z());

   // toImplementation
   ASSERT_EQ(this->velFromThreeValues.toImplementation()(0,0), this->vec1.x()) << "X-component needs to correspond to the matrix entry (0,0)!";
   ASSERT_EQ(this->velFromThreeValues.toImplementation()(1,0), this->vec1.y()) << "Y-component needs to correspond to the matrix entry (1,0)!";
   ASSERT_EQ(this->velFromThreeValues.toImplementation()(2,0), this->vec1.z()) << "Z-component needs to correspond to the matrix entry (2,0)!";

   // addition
   AngularVelocity velAdd = this->velFromEigen+this->vel2FromEigen;
   ASSERT_EQ(velAdd.x(), this->vecAdd.x());
   ASSERT_EQ(velAdd.y(), this->vecAdd.y());
   ASSERT_EQ(velAdd.z(), this->vecAdd.z());

   // addition and assignment
   AngularVelocity velAddandAssign(this->velFromEigen);
   velAddandAssign += this->vel2FromEigen;
   ASSERT_EQ(velAddandAssign.x(), this->vecAdd.x());
   ASSERT_EQ(velAddandAssign.y(), this->vecAdd.y());
   ASSERT_EQ(velAddandAssign.z(), this->vecAdd.z());

   // subtract
   AngularVelocity velSubtract = this->velFromEigen-this->vel2FromEigen;
   ASSERT_EQ(velSubtract.x(), this->vecSubtract.x());
   ASSERT_EQ(velSubtract.y(), this->vecSubtract.y());
   ASSERT_EQ(velSubtract.z(), this->vecSubtract.z());

   // subtract and assignment
   AngularVelocity velSubtractandAssign(this->velFromEigen);
   velSubtractandAssign -= this->vel2FromEigen;
   ASSERT_EQ(velSubtractandAssign.x(), this->vecSubtract.x());
   ASSERT_EQ(velSubtractandAssign.y(), this->vecSubtract.y());
   ASSERT_EQ(velSubtractandAssign.z(), this->vecSubtract.z());

}

TEST(RDiffTest, testDevelopment)
{
//  rot::RotationQuaternionPD rquat;
//
//  rot::RotationQuaternionDiffPD rquatdiff(1,2,3,4);
//  rot::AngularVelocityPD angularVelocity(rquat, rquatdiff);
//
//
//  std::cout << "rquatdiff:" << rquatdiff << std::endl;
//  std::cout << "angularVelocity:" << angularVelocity << std::endl;
//  std::cout << "angularVelocity2" << rquatdiff.cast<rot::AngularVelocityPD>(rquat) << std::endl;
//
//
//  rot::EulerAnglesXyzPD eulerAnglesXyz(0.1, 0.2, 0.3);
//  rot::AngularVelocityPD angularVelocity2(0.2, 0.2, 0.2);
//  rot::EulerAnglesXyzDiffPD eulerAnglesXyzDiff2(eulerAnglesXyz,angularVelocity2);
//
//  std::cout << "eulerDiff: " << eulerAnglesXyzDiff2 << std::endl;

//  rot::AngleAxisAD aa;
// rot::AngleAxisDiffAD aaDiff(0.2, 0.1, 0.2, 0.4);
// std::cout << "angle axis: " << aaDiff << std::endl;
// rot::AngularVelocityAD avelA6(aa, aaDiff);
//
//  rot::RotationMatrixDiffAD rmatADiff;
//  rot::RotationMatrixAD rmatA;
////  rot::AngularVelocityAD avelA(rmatA, rmatADiff);
////  std::cout << "avelA: " << avelA << std::endl;
//
//  rot::RotationMatrixDiffPD rmatPDiff;
//  rot::RotationMatrixPD rmatP;
//  rot::AngularVelocityAD avelA2(rmatA, rmatADiff);
//  rot::AngularVelocityAD avelA3(rmatP, rmatPDiff);
//  std::cout << "avelA2: " << avelA2 << std::endl;
//
//  rot::RotationVectorDiffAD rvADiff;
//  rot::RotationVectorAD rvA;
//  rot::AngularVelocityAD avelAz(rvA, rvADiff);
////  std::cout << rvA << std::endl;

  rot::AngleAxisAD aaA(0.2, 0.0, 1.0, 0.0);
  rot::AngleAxisDiffAD aaDiffA(0.5, 0.1, 0.2, 0.3);

  rot::AngularVelocityAD avA(aaA, aaDiffA);

  std::cout << "avA1 | avA2: "<<  avA << " | "  << avA2 << std::endl;

  rot::RotationQuaternionAD rqA(aaA);
  rot::RotationQuaternionDiffAD rqDiffA(rqA, avA);

  rot::AngularVelocityAD avA3(rqA, rqDiffA);

  std::cout << "avA1 | avA3: "<<  avA << " | "  << avA3 << std::endl;


}

