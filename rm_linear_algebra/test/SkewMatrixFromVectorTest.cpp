/*
 * SkewMatrixTest.cpp
 *
 *  Created on: Oct 8, 2013
 *      Author: gech
 */


#include <gtest/gtest.h>
#include <rm/linear_algebra/LinearAlgebra.hpp>

TEST (SkewMatrixFromVectorTest, testVector) {
	Eigen::Matrix3d skewMatrix;
	Eigen::Vector3d vec;
	vec << 1, 2, 3;
	skewMatrix << 0, -3, 2, 3, 0, -1, -2, 1, 0;
	EXPECT_EQ(skewMatrix, rm::linear_algebra::getSkewMatrixFromVector(vec));
}
