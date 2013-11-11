/*
 * QuaternionsTest.cpp
 *
 *  Created on: Nov 11, 2013
 *      Author: gech
 */



#include <gtest/gtest.h>

#include <rm/quaternions/QuaternionEigen.hpp>

namespace quat = rm::quaternions::eigen_implementation;


template <typename QuaternionImplementation>
struct QuaternionsTest : public ::testing::Test  {
  typedef typename QuaternionImplementation::Scalar Scalar;
  typedef Eigen::Matrix<Scalar, 3, 1> Vector3;

  QuaternionImplementation identity = QuaternionImplementation(quat::Quaternion<Scalar>(Eigen::Quaterniond(1, 0, 0, 0).cast<Scalar>()));
  QuaternionImplementation halfX =    QuaternionImplementation(quat::Quaternion<Scalar>(Eigen::Quaterniond(0, 1, 0, 0).cast<Scalar>()));
  QuaternionImplementation halfY =    QuaternionImplementation(quat::Quaternion<Scalar>(Eigen::Quaterniond(0, 0, 1, 0).cast<Scalar>()));
  QuaternionImplementation halfZ =    QuaternionImplementation(quat::Quaternion<Scalar>(Eigen::Quaterniond(0, 0, 0, 1).cast<Scalar>()));


  QuaternionsTest() : X(Vector3::UnitX()), Y(Vector3::UnitY()), Z(Vector3::UnitZ()) {}
  Vector3 X, Y, Z;
};



typedef ::testing::Types<
quat::QuaternionD,
quat::QuaternionF
> Types;

TYPED_TEST_CASE(QuaternionsTest, Types);

TYPED_TEST (QuaternionsTest, testQuatenionMultiplication ) {

//	TestFixture::QuaternionImplementation q1 = TestFixture::identity;
}
