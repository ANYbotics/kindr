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

#include "kinder/linearvelocities/LinearVelocityEigen.hpp"
#include "kinder/common/gtest_eigen.hpp"



namespace vel = kinder::linearvelocities::eigen_implementation;

template <typename LinearVelocityImplementation>
struct LinearVelocity3Test: public ::testing::Test {
  typedef LinearVelocityImplementation LinearVelocity3;
  typedef typename LinearVelocity3::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

  Scalar tol;
  Vector3 vecZero, vec1, vec2, vecAdd, vecSubtract;

  LinearVelocity3 velDefault;
  LinearVelocity3 velFromThreeValues;
  LinearVelocity3 velFromEigen;
  LinearVelocity3 vel2FromEigen;
  LinearVelocity3 velFromVel;

  LinearVelocity3Test() : tol(1e-6),
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
    vel::LinearVelocity3D,
    vel::LinearVelocity3F
> Types;


TYPED_TEST_CASE(LinearVelocity3Test, Types);

TYPED_TEST(LinearVelocity3Test, testLinearVelocity3)
{
  typedef typename TestFixture::LinearVelocity3 LinearVelocity3;

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
   LinearVelocity3 velAdd = this->velFromEigen+this->vel2FromEigen;
   ASSERT_EQ(velAdd.x(), this->vecAdd.x());
   ASSERT_EQ(velAdd.y(), this->vecAdd.y());
   ASSERT_EQ(velAdd.z(), this->vecAdd.z());

   // addition and assignment
   LinearVelocity3 velAddandAssign(this->velFromEigen);
   velAddandAssign += this->vel2FromEigen;
   ASSERT_EQ(velAddandAssign.x(), this->vecAdd.x());
   ASSERT_EQ(velAddandAssign.y(), this->vecAdd.y());
   ASSERT_EQ(velAddandAssign.z(), this->vecAdd.z());

   // subtract
   LinearVelocity3 velSubtract = this->velFromEigen-this->vel2FromEigen;
   ASSERT_EQ(velSubtract.x(), this->vecSubtract.x());
   ASSERT_EQ(velSubtract.y(), this->vecSubtract.y());
   ASSERT_EQ(velSubtract.z(), this->vecSubtract.z());

   // subtract and assignment
   LinearVelocity3 velSubtractandAssign(this->velFromEigen);
   velSubtractandAssign -= this->vel2FromEigen;
   ASSERT_EQ(velSubtractandAssign.x(), this->vecSubtract.x());
   ASSERT_EQ(velSubtractandAssign.y(), this->vecSubtract.y());
   ASSERT_EQ(velSubtractandAssign.z(), this->vecSubtract.z());

}

