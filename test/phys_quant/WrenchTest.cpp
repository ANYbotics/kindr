/*
 * Copyright (c) 2017, Christian Gehring
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

#include "kindr/phys_quant/Wrench.hpp"
#include "kindr/common/gtest_eigen.hpp"

template <typename PrimType_>
struct WrenchTest : public ::testing::Test {
  typedef kindr::Wrench<PrimType_> Wrench;
  typedef typename Wrench::Scalar Scalar;
  typedef typename Wrench::Force Force;
  typedef typename Wrench::Torque Torque;
  typedef typename Wrench::Vector3 Vector3;
};

typedef ::testing::Types<
    float,
    double
> PrimTypes;

TYPED_TEST_CASE(WrenchTest, PrimTypes);


TYPED_TEST(WrenchTest, constructor)
{
  typedef typename TestFixture::Wrench Wrench;
  typedef typename TestFixture::Force Force;
  typedef typename TestFixture::Torque Torque;
  typedef typename TestFixture::Vector3 Vector3;
  typedef typename TestFixture::Scalar Scalar;
  Wrench def;
  const Vector3 vectorZero = Vector3::Zero();
  KINDR_ASSERT_DOUBLE_MX_EQ(vectorZero, def.getForce().toImplementation(), Scalar(0.5), "def");
  KINDR_ASSERT_DOUBLE_MX_EQ(vectorZero, def.getTorque().toImplementation(), Scalar(0.5), "def");

  const Force force1(0.1, 0.2, 0.3);
  const Torque torque1(1.1, 2.2, 3.3);
  Wrench first(force1, torque1);
  KINDR_ASSERT_DOUBLE_MX_EQ(force1.toImplementation(), first.getForce().toImplementation(), Scalar(0.5), "first");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1.toImplementation(), first.getTorque().toImplementation(), Scalar(0.5), "first");

  const Vector3 forceVector1(0.1, 0.2, 0.3);
  const Vector3 torqueVector1(1.1, 2.2, 3.3);
  Wrench second(forceVector1, torqueVector1);
  KINDR_ASSERT_DOUBLE_MX_EQ(forceVector1, second.getForce().toImplementation(), Scalar(0.5), "second");
  KINDR_ASSERT_DOUBLE_MX_EQ(torqueVector1, second.getTorque().toImplementation(), Scalar(0.5), "second");

  typename Wrench::Vector6 wrenchVector;
  wrenchVector << 0.1, 0.2, 0.3, 1.1, 2.2, 3.3;
  Wrench third(wrenchVector);
  KINDR_ASSERT_DOUBLE_MX_EQ(forceVector1, third.getForce().toImplementation(), Scalar(0.5), "third");
  KINDR_ASSERT_DOUBLE_MX_EQ(torqueVector1, third.getTorque().toImplementation(), Scalar(0.5), "third");
}

TYPED_TEST(WrenchTest, setters)
{
  typedef typename TestFixture::Wrench Wrench;
  typedef typename TestFixture::Force Force;
  typedef typename TestFixture::Torque Torque;
  typedef typename TestFixture::Vector3 Vector3;
  typedef typename TestFixture::Scalar Scalar;

  const Force force1(0.1, 0.2, 0.3);
  const Torque torque1(1.1, 2.2, 3.3);
  Wrench first;
  first.setForce(force1);
  first.setTorque(torque1);
  KINDR_ASSERT_DOUBLE_MX_EQ(force1.toImplementation(), first.getForce().toImplementation(), Scalar(0.5), "first");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1.toImplementation(), first.getTorque().toImplementation(), Scalar(0.5), "first");

  const Vector3 forceVector1(0.1, 0.2, 0.3);
  const Vector3 torqueVector1(1.1, 2.2, 3.3);
  Wrench second;
  second.setForce(forceVector1);
  second.setTorque(torqueVector1);
  KINDR_ASSERT_DOUBLE_MX_EQ(forceVector1, second.getForce().toImplementation(), Scalar(0.5), "second");
  KINDR_ASSERT_DOUBLE_MX_EQ(torqueVector1, second.getTorque().toImplementation(), Scalar(0.5), "second");

  typename Wrench::Vector6 wrenchVector;
  wrenchVector << 0.1, 0.2, 0.3, 1.1, 2.2, 3.3;
  Wrench third;
  third.setVector(wrenchVector);
  KINDR_ASSERT_DOUBLE_MX_EQ(wrenchVector, third.getVector(), Scalar(0.5), "third");
}

TYPED_TEST(WrenchTest, setZero)
{
  typedef typename TestFixture::Wrench Wrench;
  typedef typename TestFixture::Force Force;
  typedef typename TestFixture::Torque Torque;
  typedef typename TestFixture::Vector3 Vector3;
  typedef typename TestFixture::Scalar Scalar;

  const Force force1(0.1, 0.2, 0.3);
  const Torque torque1(1.1, 2.2, 3.3);
  Wrench first(force1, torque1);
  first.setZero();

  const Vector3 vectorZero = Vector3::Zero();
  KINDR_ASSERT_DOUBLE_MX_EQ(vectorZero, first.getForce().toImplementation(), Scalar(0.5), "first");
  KINDR_ASSERT_DOUBLE_MX_EQ(vectorZero, first.getTorque().toImplementation(), Scalar(0.5), "first");
}

TYPED_TEST(WrenchTest, getVector)
{
  typedef typename TestFixture::Wrench Wrench;
  typedef typename TestFixture::Force Force;
  typedef typename TestFixture::Torque Torque;
  typedef typename TestFixture::Scalar Scalar;

  typename Wrench::Vector6 wrenchVector;
  wrenchVector << 0.1, 0.2, 0.3, 1.1, 2.2, 3.3;
  Wrench first(wrenchVector);

  KINDR_ASSERT_DOUBLE_MX_EQ(wrenchVector, first.getVector(), Scalar(0.5), "first");
}

TYPED_TEST(WrenchTest, assignment)
{
  typedef typename TestFixture::Wrench Wrench;
  typedef typename TestFixture::Force Force;
  typedef typename TestFixture::Torque Torque;
  typedef typename TestFixture::Vector3 Vector3;
  typedef typename TestFixture::Scalar Scalar;

  const Force force1(0.1, 0.2, 0.3);
  const Torque torque1(1.1, 2.2, 3.3);
  Wrench first(force1, torque1);
  Wrench second;
  second = first;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1.toImplementation(), second.getForce().toImplementation(), Scalar(0.5), "second");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1.toImplementation(), second.getTorque().toImplementation(), Scalar(0.5), "second");
}
TYPED_TEST(WrenchTest, comparison)
{
  typedef typename TestFixture::Wrench Wrench;
  typedef typename TestFixture::Force Force;
  typedef typename TestFixture::Torque Torque;
  typedef typename TestFixture::Vector3 Vector3;
  typedef typename TestFixture::Scalar Scalar;

  const Force force1(0.1, 0.2, 0.3);
  const Torque torque1(1.1, 2.2, 3.3);
  Wrench first(force1, torque1);
  Wrench second(force1, torque1);
  Wrench third;
  ASSERT_TRUE(first==second);
  ASSERT_FALSE(first==third);
}

TYPED_TEST(WrenchTest, math)
{
  typedef typename TestFixture::Wrench Wrench;
  typedef typename TestFixture::Force Force;
  typedef typename TestFixture::Torque Torque;
  typedef typename TestFixture::Vector3 Vector3;
  typedef typename Wrench::Scalar Scalar;

  const Force force1(0.1, 0.2, 0.3);
  const Torque torque1(1.1, 2.2, 3.3);
  Wrench wrench1(force1, torque1);

  const Force force2(0.2, 0.4, 0.6);
  const Torque torque2(2.1, 4.2, 6.3);
  Wrench wrench2(force2, torque2);

  double scalar = 5.55;

  // Addition
  const Force force1p2 = force1 + force2;
  const Torque torque1p2 = torque1 + torque2;
  Wrench wrench1p2 = wrench1 + wrench2;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1p2.toImplementation(), wrench1p2.getForce().toImplementation(), Scalar(0.5), "add");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1p2.toImplementation(), wrench1p2.getTorque().toImplementation(), Scalar(0.5), "add torque");

  // Subtraction
  const Force force1m2 = force1 - force2;
  const Torque torque1m2 = torque1 - torque2;
  Wrench wrench1m2 = wrench1 - wrench2;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1m2.toImplementation(), wrench1m2.getForce().toImplementation(), Scalar(0.5), "subtract");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1m2.toImplementation(), wrench1m2.getTorque().toImplementation(), Scalar(0.5), "subtract");

  // Multiplies with a scalar.
  const Force force1timesScalar = force1*scalar;
  const Torque torque1timesScalar = torque1*scalar;
  Wrench wrench1timesScalar = wrench1*scalar;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1timesScalar.toImplementation(), wrench1timesScalar.getForce().toImplementation(), Scalar(0.5), "multiply");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1timesScalar.toImplementation(), wrench1timesScalar.getTorque().toImplementation(), Scalar(0.5), "multiply");

  // Divides by a scalar.
  const Force force1DivideByScalar = force1/scalar;
  const Torque torque1DivideByScalar = torque1/scalar;
  Wrench wrench1DivideByScalar = wrench1/scalar;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1DivideByScalar.toImplementation(), wrench1DivideByScalar.getForce().toImplementation(), Scalar(0.5), "divide");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1DivideByScalar.toImplementation(), wrench1DivideByScalar.getTorque().toImplementation(), Scalar(0.5), "divide");

  // Addition and assignment
  Wrench wrenchAddAssing = wrench1;
  wrenchAddAssing += wrench2;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1p2.toImplementation(), wrenchAddAssing.getForce().toImplementation(), Scalar(0.5), "subtract");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1p2.toImplementation(), wrenchAddAssing.getTorque().toImplementation(), Scalar(0.5), "subtract");

  // Subtraction and assignment
  Wrench wrenchSubtractAssing = wrench1;
  wrenchSubtractAssing -= wrench2;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1m2.toImplementation(), wrenchSubtractAssing.getForce().toImplementation(), Scalar(0.5), "subtract and assign");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1m2.toImplementation(), wrenchSubtractAssing.getTorque().toImplementation(), Scalar(0.5), "subtract and assign");

  // Multiplication and assignment
  Wrench wrenchMultiplyAssign = wrench1;
  wrenchMultiplyAssign *= scalar;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1timesScalar.toImplementation(), wrenchMultiplyAssign.getForce().toImplementation(), Scalar(0.5), "multiply and assign");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1timesScalar.toImplementation(), wrenchMultiplyAssign.getTorque().toImplementation(), Scalar(0.5), "multiply and assign");

  // Division and assignment
  Wrench wrenchDivideAssign = wrench1;
  wrenchDivideAssign /= scalar;
  KINDR_ASSERT_DOUBLE_MX_EQ(force1DivideByScalar.toImplementation(), wrenchDivideAssign.getForce().toImplementation(), Scalar(0.5), "divide and assign");
  KINDR_ASSERT_DOUBLE_MX_EQ(torque1DivideByScalar.toImplementation(), wrenchDivideAssign.getTorque().toImplementation(), Scalar(0.5), "divide and assign");

}
