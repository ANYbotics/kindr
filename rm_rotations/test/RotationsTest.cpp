#include <gtest/gtest.h>

#include "rm/rotations/Rotations.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sm/eigen/gtest.hpp>
#include <sm/random.hpp>
#include <sm/timing/Timer.hpp>

#include <limits>
#include <iostream>
#include <random>

TEST (RotationsTest, testRotationMatrixFromKardanAngles ) {
  using namespace Eigen;
  using namespace rm::rotations;

  ASSERT_DOUBLE_MX_EQ(Vector3d(1,2,3), Vector3d(1,2,-3), 1e-6, "Vector3d");

//  EXPECT_EQ(2, test(2));
}
