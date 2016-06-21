/*
 * convention.cpp
 *
 *  Created on: Jun 17, 2016
 *      Author: gech
 */
#include <gtest/gtest.h>
#include <kindr/Core>
#include <kindr/rotations/gtest_rotations.hpp>

namespace kindr {


template<typename Rotation_>
class RotationConversion<Rotation_, Eigen::Matrix<typename Rotation_::Scalar, 3,1>, typename Rotation_::Scalar> {
  typedef typename Rotation_::Scalar Scalar;
  typedef Rotation_ Rotation;
  typedef Eigen::Matrix<Scalar, 3,1> Vector;
 public:
  inline static void convertToOtherRotation(Rotation& otherRotation, const kindr::RotationQuaternion<Scalar>& quaternionIn) {
    // Implement quaternionOut = f(quaternionIn);
    otherRotation = Rotation(quaternionIn);
  }

  inline static void convertToKindr(kindr::RotationQuaternion<Scalar>& quaternion, const Rotation& otherRotation) {
    // Implement quaternionOut = f(quaternionIn);
    quaternion = kindr::RotationQuaternion<Scalar>(otherRotation);
  }

  inline static void convertToOtherVelocityVector(Vector& velocityOut, const Rotation& rot, const Eigen::Matrix<Scalar,3,1>& velocityIn) {
    // Implement velocityOut = g(velocityIn, rot);
    velocityOut = velocityIn;
  }

  inline static void getRotationMatrixFromRotation(Eigen::Matrix<Scalar,3,3>& rotationMatrix, const Rotation& rotation) {
    // Implement rotationMatrix = C(quaternion);
    rotationMatrix = kindr::RotationMatrix<Scalar>(rotation).matrix();
  }

  inline static void concatenate(Rotation& res,  const Rotation& rot1,  const Rotation& rot2) {
    res = rot2*rot1;
  }

  inline static void rotateVector(Eigen::Matrix<Scalar,3,1>& A_r, const Rotation& rotation, const Eigen::Matrix<Scalar,3,1>& B_r) {
    A_r = rotation.rotate(B_r);
  }

  inline static void boxPlus(Rotation& res, const Rotation& rotation, const Vector& velocity) {
    // Implement res = rotation.boxPlus(vector);
    res = rotation.boxPlus(velocity);
  }

  inline static bool testRotation(const Rotation& expected, const Rotation& actual) {
    // Implement EXPECT_NEAR(expected, actual, 1.0e-6);
    return expected.isNear(actual, 1.0e-3);
  }
};

}

template <typename Rotation_>
struct KindrConventionTest : public ::testing::Test{
  typedef typename Rotation_::Scalar Scalar;
  typedef Rotation_ Rotation;
};

typedef ::testing::Types<
    kindr::RotationQuaternionD,
    kindr::EulerAnglesZyxD,
    kindr::EulerAnglesXyzD,
    kindr::RotationMatrixD,
    kindr::RotationVectorD,
    kindr::AngleAxisD
> Types;

TYPED_TEST_CASE(KindrConventionTest, Types);

TYPED_TEST(KindrConventionTest, GeometricalInterpretation)
{
 typedef typename TestFixture::Scalar Scalar;
 typedef typename TestFixture::Rotation Rotation;
 kindr::ConventionTest<Rotation, Eigen::Matrix<Scalar, 3,1>, Scalar>::testGeometricalInterpretation();
}

TYPED_TEST(KindrConventionTest, Concatenation)
{
 typedef typename TestFixture::Scalar Scalar;
 typedef typename TestFixture::Rotation Rotation;
 kindr::ConventionTest<Rotation, Eigen::Matrix<Scalar, 3,1>, Scalar>::testConcatenation();
}

TYPED_TEST(KindrConventionTest, BoxPlus)
{
 typedef typename TestFixture::Scalar Scalar;
 typedef typename TestFixture::Rotation Rotation;
 kindr::ConventionTest<Rotation, Eigen::Matrix<Scalar, 3,1>, Scalar>::testBoxPlus();
}

TYPED_TEST(KindrConventionTest, RotationMatrix)
{
 typedef typename TestFixture::Scalar Scalar;
 typedef typename TestFixture::Rotation Rotation;
 kindr::ConventionTest<Rotation, Eigen::Matrix<Scalar, 3,1>, Scalar>::testRotationMatrix();
}


