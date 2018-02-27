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

#include "kindr/phys_quant/PhysicalQuantities.hpp"
#include "kindr/common/gtest_eigen.hpp"

namespace pos = kindr;

template <typename LinearVelocityImplementation>
struct LinearVelocityTest: public ::testing::Test {
  typedef LinearVelocityImplementation LinearVelocity;
  typedef typename LinearVelocity::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

  Scalar tol;
  Scalar factor;
  Scalar divisor;
  Vector3 vecZero, vec1, vec2, vecAdd, vecSubtract, vecMult, vecDiv;

  LinearVelocity velDefault;
  LinearVelocity velFromThreeValues;
  LinearVelocity velFromEigen;
  LinearVelocity vel2FromEigen;
  LinearVelocity velFromVel;

  LinearVelocityTest() : tol(1e-6),
      factor(2),
      divisor(2),
      vecZero(Vector3::Zero()),
      vec1(10,20,30),
      vec2(1,2,3),
      vecAdd(11,22,33),
      vecSubtract(9,18,27),
      vecMult(20,40,60),
      vecDiv(5,10,15),
      velFromThreeValues(vec1.x(), vec1.y(), vec1.z()),
      velFromEigen(vec1),
      vel2FromEigen(vec2),
      velFromVel(velFromEigen)
  {}
};


typedef ::testing::Types<
    pos::Velocity3D,
    pos::Velocity3F
> Types;


TYPED_TEST_CASE(LinearVelocityTest, Types);

TYPED_TEST(LinearVelocityTest, testLinearVelocity)
{
  typedef typename TestFixture::LinearVelocity LinearVelocity;

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
   LinearVelocity velAdd = this->velFromEigen+this->vel2FromEigen;
   ASSERT_EQ(velAdd.x(), this->vecAdd.x());
   ASSERT_EQ(velAdd.y(), this->vecAdd.y());
   ASSERT_EQ(velAdd.z(), this->vecAdd.z());

   // addition and assignment
   LinearVelocity velAddandAssign(this->velFromEigen);
   velAddandAssign += this->vel2FromEigen;
   ASSERT_EQ(velAddandAssign.x(), this->vecAdd.x());
   ASSERT_EQ(velAddandAssign.y(), this->vecAdd.y());
   ASSERT_EQ(velAddandAssign.z(), this->vecAdd.z());

   // subtract
   LinearVelocity velSubtract = this->velFromEigen-this->vel2FromEigen;
   ASSERT_EQ(velSubtract.x(), this->vecSubtract.x());
   ASSERT_EQ(velSubtract.y(), this->vecSubtract.y());
   ASSERT_EQ(velSubtract.z(), this->vecSubtract.z());

   // subtract and assignment
   LinearVelocity velSubtractandAssign(this->velFromEigen);
   velSubtractandAssign -= this->vel2FromEigen;
   ASSERT_EQ(velSubtractandAssign.x(), this->vecSubtract.x());
   ASSERT_EQ(velSubtractandAssign.y(), this->vecSubtract.y());
   ASSERT_EQ(velSubtractandAssign.z(), this->vecSubtract.z());

   // multiplication a)
   LinearVelocity velMultA = this->velFromEigen * this->factor;
   ASSERT_EQ(velMultA.x(), this->vecMult.x());
   ASSERT_EQ(velMultA.y(), this->vecMult.y());
   ASSERT_EQ(velMultA.z(), this->vecMult.z());

   // multiplication b)
   LinearVelocity velMultB = this->factor * this->velFromEigen;
   ASSERT_EQ(velMultB.x(), this->vecMult.x());
   ASSERT_EQ(velMultB.y(), this->vecMult.y());
   ASSERT_EQ(velMultB.z(), this->vecMult.z());

   // multiplication and assignment
   LinearVelocity velMultAndAssign(this->velFromEigen);
   velMultAndAssign *= this->factor;
   ASSERT_EQ(velMultAndAssign.x(), this->vecMult.x());
   ASSERT_EQ(velMultAndAssign.y(), this->vecMult.y());
   ASSERT_EQ(velMultAndAssign.z(), this->vecMult.z());

   // division
   LinearVelocity velDiv = this->velFromEigen / this->divisor;
   ASSERT_EQ(velDiv.x(), this->vecDiv.x());
   ASSERT_EQ(velDiv.y(), this->vecDiv.y());
   ASSERT_EQ(velDiv.z(), this->vecDiv.z());

   // division and assignment
   LinearVelocity velDivAndAssign(this->velFromEigen);
   velDivAndAssign /= this->divisor;
   ASSERT_EQ(velDivAndAssign.x(), this->vecDiv.x());
   ASSERT_EQ(velDivAndAssign.y(), this->vecDiv.y());
   ASSERT_EQ(velDivAndAssign.z(), this->vecDiv.z());
}

