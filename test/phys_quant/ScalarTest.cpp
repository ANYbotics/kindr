#include <gtest/gtest.h>

#include "kindr/common/gtest_eigen.hpp"
#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/phys_quant/PhysicalQuantities.hpp"
#include "kindr/vectors/Vector.hpp"

struct ScalarTest : public ::testing::Test {
  using Acceleration = kindr::Acceleration3D;
  using Force = kindr::Force3D;
  using Mass = kindr::MassD;

  Acceleration acceleration;
  Force force;
  Mass mass;

  ScalarTest() : acceleration(1., 2., 3.), force(5., 10., 15.), mass(5.) {}
};

TEST_F(ScalarTest, multiplicationWithScalar) {
  EXPECT_EQ(acceleration * mass, force);
  EXPECT_EQ(mass * acceleration, force);
}

TEST_F(ScalarTest, divisionByScalar) {
  EXPECT_EQ(force / mass, acceleration);
}

TEST_F(ScalarTest, multiplyTwoScalars) {
  kindr::Acceleration<double, 1> accelerationScalar(2.);
  kindr::Force<double, 1> forceScalar(10.);

  EXPECT_EQ(accelerationScalar * mass, forceScalar);
  EXPECT_EQ(mass * accelerationScalar, forceScalar);
}

TEST_F(ScalarTest, divideTwoScalars) {
  kindr::Acceleration<double, 1> accelerationScalar(2.);
  kindr::Force<double, 1> forceScalar(10.);

  EXPECT_EQ(forceScalar / mass, accelerationScalar);
  EXPECT_EQ(forceScalar / accelerationScalar, mass);
}

TEST_F(ScalarTest, assignment) {
  const kindr::VectorTypeless<double, 1> typeless(5.);

  acceleration *= typeless;
  EXPECT_EQ(acceleration, kindr::Acceleration3D(5., 10., 15.));

  force /= typeless;
  EXPECT_EQ(force, kindr::Force3D(1., 2., 3.));
}
