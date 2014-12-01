#include <cmath>
#include <gtest/gtest.h>
#include <kindr/core/functional/All.hpp>

using namespace kindr::core;
using namespace kindr::core::functional;

const Vector3d zero = Vector3d::Zero();
const Vector3d x = Vector3d::UnitX();
const Vector3d y = Vector3d::UnitY();
const Vector3d z = Vector3d::UnitZ();
const Matrix4d MatId4x4 = Matrix4d::Identity();

const double halfPi = M_PI / 2.0;

TEST(FunctionalTest, testCompileAndShowcase) {
  // construct
  SE3Mat4D::Storage se3Id = SE3Mat4D::exp(zero);
  SE3Mat4D::Storage se3Id2(MatId4x4);

  T3VecD::Storage transX(x);
  AngleAxisD::Storage angleAxisHalfPiX({halfPi, x});
  AngleAxisD::Storage angleAxisOneX = AngleAxisD::exp(x);

  // conversion
  SO3MatD::Storage R1 = convertFromTo<AngleAxisD, SO3MatD>(angleAxisHalfPiX);

  // access data
  EXPECT_EQ(1, angleAxisOneX.angle);
  EXPECT_EQ(x, angleAxisOneX.axis);
  EXPECT_EQ(MatId4x4, se3Id);
  EXPECT_EQ(MatId4x4, se3Id2);

  // apply se3Id:
  EXPECT_EQ(x, SE3Mat4D::apply(se3Id, x));
  EXPECT_EQ(x, AngleAxisD::apply(angleAxisOneX, x));
  EXPECT_EQ(Vector3<double>(x + y), T3VecD::apply(transX, y));
  EXPECT_NEAR((z - AngleAxisD::apply(angleAxisHalfPiX, y)).norm(), 0, 1e-9);

  // compose
  EXPECT_EQ(MatId4x4, SE3Mat4D::compose(se3Id , se3Id));
//  EXPECT_EQ(SO3AfterT3<AngleAxisD>(angleAxisHalfPiX, transX), angleAxisHalfPiX * transX); // an experimental support for composition across different SE3 types
//  EXPECT_NEAR((x + z - (angleAxisHalfPiX * transX).apply(y)).norm(), 0, 1e-9);
}
