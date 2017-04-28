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

#include "kindr/common/gtest_eigen.hpp"
#include "kindr/quaternions/Quaternion.hpp"
#include <string>

#include "kindr/rotations/Rotation.hpp"
#include "kindr/common/assert_macros_eigen.hpp"


namespace rot = kindr;
namespace quat = kindr;



template <typename ImplementationPair_>
struct ConversionTest : public ::testing::Test{
  typedef typename ImplementationPair_::first_type RotationA;
  typedef typename ImplementationPair_::second_type RotationB;
  typedef typename RotationB::Scalar Scalar;
  RotationA rotA;
  RotationB rotB;

  double tol = 1.0e-3;

  void convertAToB() {
    this->rotB.rot = this->rotA.rotIdentity;
    this->rotB.assertNear(this->rotB.rotIdentity, this->rotB.rot, this->tol, "Identity");

    this->rotB.rot = this->rotA.rotQuarterX;
    this->rotB.assertNear(this->rotB.rotQuarterX, this->rotB.rot, this->tol, "QuarterX");

    this->rotB.rot = this->rotA.rotQuarterY;
    this->rotB.assertNear(this->rotB.rotQuarterY, this->rotB.rot, this->tol, "QuarterY");

    this->rotB.rot = this->rotA.rotQuarterZ;
    this->rotB.assertNear(this->rotB.rotQuarterZ, this->rotB.rot, this->tol, "QuarterZ");

  }

  void convertBToA() {
    this->rotA.rot = this->rotB.rotIdentity;
    this->rotA.assertNear(this->rotA.rotIdentity, this->rotA.rot, this->tol, "Identity");

    this->rotA.rot = this->rotB.rotQuarterX;
    this->rotA.assertNear(this->rotA.rotQuarterX, this->rotA.rot, this->tol, "QuarterX");

    this->rotA.rot = this->rotB.rotQuarterY;
    this->rotA.assertNear(this->rotA.rotQuarterY, this->rotA.rot, this->tol, "QuarterY");

    this->rotA.rot = this->rotB.rotQuarterZ;
    this->rotA.assertNear(this->rotA.rotQuarterZ, this->rotA.rot, this->tol, "QuarterZ");
  }
};

template <typename ImplementationPair_>
struct Conversion2Test : public ConversionTest<ImplementationPair_> {

};

template <typename ImplementationPair_>
struct Conversion3Test : public ConversionTest<ImplementationPair_> {

};

template <typename ImplementationPair_>
struct Conversion4Test : public ConversionTest<ImplementationPair_> {

};

template <typename ImplementationPair_>
struct ConcatenationTest : public ConversionTest<ImplementationPair_> {

};

template <typename Rotation_>
struct RotationQuaternionTestType {
  typedef Rotation_ Rotation;
  typedef typename Rotation::Scalar Scalar;

  const Rotation rotQuarterX = Rotation(1/sqrt(2.0),1/sqrt(2.0),0.0,0.0);
  const Rotation rotQuarterY = Rotation(1/sqrt(2.0),0.0,1/sqrt(2.0),0.0);
  const Rotation rotQuarterZ = Rotation(1/sqrt(2.0),0.0,0.0,1/sqrt(2.0));
  const Rotation rotIdentity = Rotation(1.0,0.0,0.0,0.0);
  const Rotation rotGeneric = Rotation(quat::Quaternion<Scalar>(2.0, 3.0, 4.0, 5.0).toUnitQuaternion());

  Rotation rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-6, const std::string& msg = "") {
    ASSERT_NEAR(rotA.w(), rotB.w(), tol) << msg;
    ASSERT_NEAR(rotA.x(), rotB.x(), tol) << msg;
    ASSERT_NEAR(rotA.y(), rotB.y(), tol) << msg;
    ASSERT_NEAR(rotA.z(), rotB.z(), tol) << msg;
  }
};


template <typename Rotation_>
struct RotationVectorTestType {
  typedef Rotation_ Rotation;
  typedef typename Rotation::Scalar Scalar;

  const Rotation rotQuarterX = Rotation(M_PI/2.0,0.0,0.0);
  const Rotation rotQuarterY = Rotation(0.0,M_PI/2.0,0.0);
  const Rotation rotQuarterZ = Rotation(0.0,0.0,M_PI/2.0);
  const Rotation rotIdentity = Rotation(0.0,0.0,0.0);
  const Rotation rotGeneric = Rotation(2.0,3.0,4.0);

  Rotation rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-6, const std::string& msg = "") {
    ASSERT_NEAR(rotA.x(), rotB.x(), tol) << msg;
    ASSERT_NEAR(rotA.y(), rotB.y(), tol) << msg;
    ASSERT_NEAR(rotA.z(), rotB.z(), tol) << msg;
  }
};

template <typename Rotation_>
struct AngleAxisTestType {
  typedef Rotation_ Rotation;
  typedef typename Rotation::Scalar Scalar;

  const Rotation rotQuarterX = Rotation(M_PI/2.0, 1.0, 0.0, 0.0);
  const Rotation rotQuarterY = Rotation(M_PI/2.0, 0.0, 1.0, 0.0);
  const Rotation rotQuarterZ = Rotation(M_PI/2.0, 0.0, 0.0, 1.0);
  const Rotation rotIdentity = Rotation(0.0, 1.0, 0.0, 0.0);
  const Rotation rotGeneric = Rotation(2.0, typename Rotation::Vector3(3.0, 4.0, 5.0).normalized());

  Rotation rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-6, const std::string& msg = "") {
    /* Due to changes in Eigen's implementation of the conversion from unit quaternions to the
     * angle axis representation, we compare the rotational vector representation of rotations A and B.
     * In particular, the tolerance on the angle of the rotation representation has been lowered from
     *      NumTraits<Scalar>::dummy_precision()*NumTraits<Scalar>::dummy_precision()
     * to Scalar(0).
     *
     * Reference:
     *  Eigen 3.2   http://eigen.tuxfamily.org/dox-3.2/AngleAxis_8h_source.html, line 173
     *  Eigen 3.3   http://eigen.tuxfamily.org/dox/AngleAxis_8h_source.html, line 178
     */
    kindr::RotationVector<Scalar> vecA(rotA.axis()*rotA.angle());
    kindr::RotationVector<Scalar> vecB(rotB.axis()*rotB.angle());

    ASSERT_NEAR(vecA.x(), vecB.x(), tol) << msg;
    ASSERT_NEAR(vecA.y(), vecB.y(), tol) << msg;
    ASSERT_NEAR(vecA.z(), vecB.z(), tol) << msg;
  }
};

template <typename Rotation_>
struct EulerAnglesZyxTestType {
  typedef Rotation_ Rotation;
  typedef typename Rotation::Scalar Scalar;

  const Rotation rotQuarterX = Rotation(0.0, 0.0, M_PI/2.0);
  const Rotation rotQuarterY = Rotation(0.0, M_PI/2.0, 0.0);
  const Rotation rotQuarterZ = Rotation(M_PI/2.0, 0.0, 0.0);
  const Rotation rotIdentity = Rotation(0.0, 0.0, 0.0);
  const Rotation rotGeneric = Rotation(0.2, 0.3, 0.4);

  Rotation rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-6, const std::string& msg = "") {
    kindr::RotationQuaternion<Scalar> quatA(rotA);
    kindr::RotationQuaternion<Scalar> quatB(rotB);
    ASSERT_TRUE(quatA.isNear(quatB, 1.0e-3))<< std::endl << msg << "\n quatA: " << quatA << " quatB: " << quatB;
  }
};

template <typename Rotation_>
struct EulerAnglesXyzTestType {
  typedef Rotation_ Rotation;
  typedef typename Rotation::Scalar Scalar;

  const Rotation rotQuarterX = Rotation(M_PI/2.0, 0.0, 0.0);
  const Rotation rotQuarterY = Rotation(0.0, M_PI/2.0, 0.0);
  const Rotation rotQuarterZ = Rotation(0.0, 0.0, M_PI/2.0);
  const Rotation rotIdentity = Rotation(0.0, 0.0, 0.0);
  const Rotation rotGeneric = Rotation(0.2, 0.3, 0.4);

  Rotation rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, double tol=1e-6, const std::string& msg = "") {

    ASSERT_EQ(true, (kindr::compareRelativePeriodic(rotA.roll(), rotB.roll(), 2*M_PI, 1e-1) &&
                     kindr::compareRelativePeriodic(rotA.pitch(), rotB.pitch(), 2*M_PI, 1e-1) &&
                     kindr::compareRelativePeriodic(rotA.yaw(), rotB.yaw(), 2*M_PI, 1e-1)) ||
                    (kindr::compareRelativePeriodic(rotA.getUnique().roll(), rotB.getUnique().roll(), 2*M_PI, 1e-1) &&
                     kindr::compareRelativePeriodic(rotA.getUnique().pitch(), rotB.getUnique().pitch(), 2*M_PI, 1e-1) &&
                     kindr::compareRelativePeriodic(rotA.getUnique().yaw(), rotB.getUnique().yaw(), 2*M_PI, 1e-1))
    ) << std::endl << msg;

  }
};



template <typename Rotation_>
struct RotationMatrixTestType {
  typedef Rotation_ Rotation;
  typedef typename Rotation::Scalar Scalar;

  Rotation rotQuarterX;
  Rotation rotQuarterY;
  Rotation rotQuarterZ;
  Rotation rotIdentity;
  Rotation rotGeneric;

  Rotation rot;

  void assertNear(const Rotation_& rotA, const Rotation_& rotB, Scalar tol=Scalar(1e-4), const std::string& msg = "") {
    KINDR_ASSERT_DOUBLE_MX_EQ(rotA.toImplementation(), rotB.toImplementation(), tol, msg);
  }

  RotationMatrixTestType() {
    rotQuarterX = Rotation( 1.0,  0.0,  0.0,
                            0.0,  0.0,  -1.0,
                            0.0, 1.0,  0.0); // psi=0, theta=0, phi=pi/2

    rotQuarterY = Rotation( 0.0,  0.0,  1.0,
                          0.0,  1.0,  0.0,
                         -1.0,  0.0,  0.0); // psi=0, theta=pi/2, phi=0

    rotQuarterZ = Rotation( 0.0,  -1.0,  0.0,
                            1.0,  0.0,  0.0,
                              0.0,  0.0,  1.0); // psi=pi/2, theta=0, phi=0

    rotIdentity = Rotation( 1.0,  0.0,  0.0,
                            0.0,  1.0,  0.0,
                            0.0,  0.0,  1.0);

    rotGeneric = Rotation(879.923176281257e-003,    372.025551942260e-003,   -295.520206661340e-003,
                           -327.579672728226e-003,    925.564159446682e-003,    189.796060978687e-003,
                            344.131896020075e-003,   -70.1995402393384e-003,    936.293363584199e-003); //psi=0.4, theta=0.3 phi=0.2

  }
};


typedef ::testing::Types<
    // Rotation Quaternion <-> Rotation Vector
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationVectorTestType<rot::RotationVectorPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationVectorTestType<rot::RotationVectorPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationVectorTestType<rot::RotationVectorPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationVectorTestType<rot::RotationVectorPD>>,

    // Rotation Quaternion <-> Rotation Matrix
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationMatrixTestType<rot::RotationMatrixPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationMatrixTestType<rot::RotationMatrixPD>>,

    // Rotation Quaternion <-> Angle-Axis
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, AngleAxisTestType<rot::AngleAxisPD>>,

    // Rotation Vector <-> Rotation Matrix
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, RotationMatrixTestType<rot::RotationMatrixPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, RotationMatrixTestType<rot::RotationMatrixPD>>,

    // Rotation Vector <-> Angle-Axis
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, AngleAxisTestType<rot::AngleAxisPD>>,

    // Rotation Matrix <-> Angle-Axis
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, AngleAxisTestType<rot::AngleAxisPD>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, AngleAxisTestType<rot::AngleAxisPD>>

> TypeRotationPairs;

typedef ::testing::Types<
    // Rotation Quaternion <-> Euler Angles ZYX
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,

    // Rotation Vector <-> Euler Angles ZYX
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,

    // Angle-Axis <-> Euler Angles ZYX
    std::pair<AngleAxisTestType<rot::AngleAxisPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,

    // Rotation Matrix <-> Euler Angles ZYX
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>

> TypeRotation2Pairs;

typedef ::testing::Types<
    // Rotation Quaternion <-> Euler Angles XYZ
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,

    // Rotation Vector <-> Euler Angles XYZ
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,

    // Angle-Axis <-> Euler Angles XYZ
    std::pair<AngleAxisTestType<rot::AngleAxisPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,

    // Rotation Matrix <-> Euler Angles XYZ
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,

    // Euler Angles ZYX <-> Euler Angles XYZ
    std::pair<EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,
    std::pair<EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>
> TypeRotation3Pairs;


typedef ::testing::Types<
    // Rotation Quaternion <-> Euler Angles Zyx
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxD>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxD>>,

    // Rotation Vector <-> Euler Angles Zyx
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,

    // Angle-Axis <-> Euler Angles Zyx
    std::pair<AngleAxisTestType<rot::AngleAxisPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxF>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxD>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxF>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxD>>,

    // Rotation Matrix <-> Euler Angles Zyx
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,

    // Euler Angles ZYX <-> Euler Angles XYZ
    std::pair<EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>,
    std::pair<EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPF>>,
    std::pair<EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>, EulerAnglesXyzTestType<rot::EulerAnglesXyzPD>>

> TypeRotation4Pairs;

typedef ::testing::Types<
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationVectorTestType<rot::RotationVectorPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationVectorTestType<rot::RotationVectorPD>>,

    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, RotationMatrixTestType<rot::RotationMatrixPD>>,

    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, AngleAxisTestType<rot::AngleAxisPD>>,

    std::pair<RotationVectorTestType<rot::RotationVectorPF>, RotationMatrixTestType<rot::RotationMatrixPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, RotationMatrixTestType<rot::RotationMatrixPD>>,

    std::pair<RotationVectorTestType<rot::RotationVectorPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, AngleAxisTestType<rot::AngleAxisPD>>,

    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, AngleAxisTestType<rot::AngleAxisPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, AngleAxisTestType<rot::AngleAxisPD>>,

    // Rotation Quaternion <-> Euler Angles ZYX
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationQuaternionTestType<rot::RotationQuaternionPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,

    // Rotation Vector <-> Euler Angles ZYX
    std::pair<RotationVectorTestType<rot::RotationVectorPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationVectorTestType<rot::RotationVectorPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,

    // Angle-Axis <-> Euler Angles ZYX
    std::pair<AngleAxisTestType<rot::AngleAxisPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<AngleAxisTestType<rot::AngleAxisPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>,

    // Rotation Matrix <-> Euler Angles ZYX
    std::pair<RotationMatrixTestType<rot::RotationMatrixPF>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPF>>,
    std::pair<RotationMatrixTestType<rot::RotationMatrixPD>, EulerAnglesZyxTestType<rot::EulerAnglesZyxPD>>

> TypeRotationPrimTypePairs;

TYPED_TEST_CASE(ConversionTest, TypeRotationPairs);
TYPED_TEST_CASE(Conversion2Test, TypeRotation2Pairs);
TYPED_TEST_CASE(Conversion3Test, TypeRotation3Pairs);
TYPED_TEST_CASE(Conversion4Test, TypeRotation4Pairs);
TYPED_TEST_CASE(ConcatenationTest, TypeRotationPrimTypePairs);

TYPED_TEST(ConversionTest, testAToB) {
  this->convertAToB();
}

TYPED_TEST(Conversion2Test, testAToB) {
  this->convertAToB();
}

TYPED_TEST(Conversion3Test, testAToB) {
  this->convertAToB();
}

TYPED_TEST(Conversion4Test, testAToB) {
  this->convertAToB();
}

TYPED_TEST(ConversionTest, testBToA) {
  this->convertBToA();
}

TYPED_TEST(Conversion2Test, testBToA) {
  this->convertBToA();
}

TYPED_TEST(Conversion3Test, testBToA) {
  this->convertBToA();
}

TYPED_TEST(Conversion4Test, testBToA) {
  this->convertBToA();
}


TYPED_TEST(ConcatenationTest, testAToB) {
  typedef typename TestFixture::Scalar Scalar;
  // Check result of multiplication of a generic rotation with identity
  this->rotB.rot = this->rotB.rotGeneric*this->rotA.rotIdentity;
  this->rotB.assertNear(this->rotB.rotGeneric.getUnique(), this->rotB.rot.getUnique(), this->tol, "rhs: identity");

  this->rotB.rot = this->rotA.rotIdentity*this->rotB.rotGeneric;
  this->rotB.assertNear(this->rotB.rotGeneric.getUnique(), this->rotB.rot.getUnique(), this->tol, "lhs: identity");

  // Check concatenation of 4 quarters
  this->rotB.rot = this->rotA.rotQuarterX*this->rotB.rotQuarterX*this->rotA.rotQuarterX*this->rotB.rotQuarterX;
  this->rotB.assertNear(this->rotB.rotIdentity, this->rotB.rot.getUnique(), this->tol, "4 quarters X");

  this->rotB.rot = this->rotA.rotQuarterY*this->rotB.rotQuarterY*this->rotA.rotQuarterY*this->rotB.rotQuarterY;
  this->rotB.assertNear(this->rotB.rotIdentity, this->rotB.rot.getUnique(), this->tol, "4 quarters Y");

  this->rotB.rot = this->rotA.rotQuarterZ*this->rotB.rotQuarterZ*this->rotA.rotQuarterZ*this->rotB.rotQuarterZ;
  this->rotB.assertNear(this->rotB.rotIdentity, this->rotB.rot.getUnique(), this->tol, "4 quarters Z");

  // check concatenation of 3 different quarters
  this->rotB.rot = this->rotB.rotQuarterX.inverted()*this->rotA.rotQuarterY*this->rotB.rotQuarterX;
  this->rotB.assertNear(this->rotB.rotQuarterZ.inverted(), this->rotB.rot, this->tol, "concatenation 1");

  this->rotB.rot = this->rotB.rotQuarterX.inverted()*this->rotA.rotQuarterZ*this->rotB.rotQuarterX;
  this->rotB.assertNear(this->rotB.rotQuarterY, this->rotB.rot, this->tol, "concatenation 2");

  this->rotB.rot = this->rotB.rotQuarterY.inverted()*this->rotA.rotQuarterX*this->rotB.rotQuarterY;
  this->rotB.assertNear(this->rotB.rotQuarterZ, this->rotB.rot, this->tol, "concatenation 3");

  this->rotB.rot = this->rotB.rotQuarterY.inverted()*this->rotA.rotQuarterZ*this->rotB.rotQuarterY;
  this->rotB.assertNear(this->rotB.rotQuarterX.inverted(), this->rotB.rot, this->tol, "concatenation 4");

  this->rotB.rot = this->rotB.rotQuarterZ.inverted()*this->rotA.rotQuarterX*this->rotB.rotQuarterZ;
  this->rotB.assertNear(this->rotB.rotQuarterY.inverted(), this->rotB.rot, this->tol, "concatenation 5");

  this->rotB.rot = this->rotB.rotQuarterZ.inverted()*this->rotA.rotQuarterY*this->rotB.rotQuarterZ;
  this->rotB.assertNear(this->rotB.rotQuarterX, this->rotB.rot, this->tol, "concatenation 6");
}



TYPED_TEST(ConcatenationTest, testBToA) {

  // Check result of multiplication of a generic rotation with identity
  this->rotA.rot = this->rotA.rotGeneric*this->rotB.rotIdentity;
  this->rotA.assertNear(this->rotA.rotGeneric.getUnique(), this->rotA.rot.getUnique(), this->tol, "rhs: identity");

  this->rotA.rot = this->rotB.rotIdentity*this->rotA.rotGeneric;
  this->rotA.assertNear(this->rotA.rotGeneric.getUnique(), this->rotA.rot.getUnique(), this->tol, "lhs: identity");

  // Check concatenation of 4 quarters
  this->rotA.rot = this->rotB.rotQuarterX*this->rotA.rotQuarterX*this->rotB.rotQuarterX*this->rotA.rotQuarterX;
  this->rotA.assertNear(this->rotA.rotIdentity, this->rotA.rot.getUnique(), this->tol, "4 quarters X");

  this->rotA.rot = this->rotB.rotQuarterY*this->rotA.rotQuarterY*this->rotB.rotQuarterY*this->rotA.rotQuarterY;
  this->rotA.assertNear(this->rotA.rotIdentity, this->rotA.rot.getUnique(), this->tol, "4 quarters Y");

  this->rotA.rot = this->rotB.rotQuarterZ*this->rotA.rotQuarterZ*this->rotB.rotQuarterZ*this->rotA.rotQuarterZ;
  this->rotA.assertNear(this->rotA.rotIdentity, this->rotA.rot.getUnique(), this->tol, "4 quarters Z");


  // Check concatenation of 3 different quarters
  this->rotA.rot = this->rotA.rotQuarterX.inverted()*this->rotB.rotQuarterY*this->rotA.rotQuarterX;
  this->rotA.assertNear(this->rotA.rotQuarterZ.inverted(), this->rotA.rot.getUnique(), this->tol, "concatenation 1");

  this->rotA.rot = this->rotA.rotQuarterX.inverted()*this->rotB.rotQuarterZ*this->rotA.rotQuarterX;
  this->rotA.assertNear(this->rotA.rotQuarterY, this->rotA.rot.getUnique(), this->tol, "concatenation 2");

  this->rotA.rot = this->rotA.rotQuarterY.inverted()*this->rotB.rotQuarterX*this->rotA.rotQuarterY;
  this->rotA.assertNear(this->rotA.rotQuarterZ, this->rotA.rot.getUnique(), this->tol, "concatenation 3");

  this->rotA.rot = this->rotA.rotQuarterY.inverted()*this->rotB.rotQuarterZ*this->rotA.rotQuarterY;
  this->rotA.assertNear(this->rotA.rotQuarterX.inverted(), this->rotA.rot.getUnique(), this->tol, "concatenation 4");

  this->rotA.rot = this->rotA.rotQuarterZ.inverted()*this->rotB.rotQuarterX*this->rotA.rotQuarterZ;
  this->rotA.assertNear(this->rotA.rotQuarterY.inverted(), this->rotA.rot.getUnique(), this->tol, "concatenation 5");

  this->rotA.rot = this->rotA.rotQuarterZ.inverted()*this->rotB.rotQuarterY*this->rotA.rotQuarterZ;
  this->rotA.assertNear(this->rotA.rotQuarterX, this->rotA.rot.getUnique(), this->tol, "concatenation 6");


}

