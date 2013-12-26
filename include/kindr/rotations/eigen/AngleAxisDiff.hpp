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
#ifndef KINDR_ROTATIONS_EIGEN_ANGLEAXIS_DIFF_HPP_
#define KINDR_ROTATIONS_EIGEN_ANGLEAXIS_DIFF_HPP_

#include <Eigen/Core>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationDiffBase.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include "kindr/linear_algebra/LinearAlgebra.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {


/*! \class AngleAxisDiff
 * \brief Time derivative of an angle-axis.
 *
 * This class implements the time derivative of an angle-axis using a Eigen::AngleAxis<Scalar> as data storage.
 *
 * \tparam PrimType_  Primitive data type of the coordinates.
 * \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class AngleAxisDiff : public AngleAxisDiffBase<AngleAxisDiff<PrimType_, Usage_>,Usage_>{
 public:

  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief The axis type is a 3D vector.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Vector3;

  /*! \brief All four parameters stored in a vector [angle; axis]
   */
  typedef Eigen::Matrix<PrimType_, 4, 1> Vector4;

  /*! \brief Default constructor sets all derivatives to zero
   */
  AngleAxisDiff()
    : angle_(0), axis_(Vector3::Zero()) {
  }


  /*! \brief Constructor using four scalars.
   *  \param angle   time derivative of the rotation angle
   *  \param v1      first entry of the time derivative of the rotation axis vector
   *  \param v2      second entry of the time derivative of the rotation axis vector
   *  \param v3      third entry of the time derivative of the rotation axis vector
   */
  AngleAxisDiff(Scalar angle, Scalar v1, Scalar v2, Scalar v3)
    : angle_(angle),axis_(v1,v2,v3) {
  }

  AngleAxisDiff(Scalar angle, const Vector3& axis)
    : angle_(angle), axis_(axis) {
  }

  AngleAxisDiff(const Vector4& vector4)
    : angle_(vector4(0)), axis_(vector4(1), vector4(2), vector4(3)) {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit AngleAxisDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RotationDiffBase<OtherDerived_, Usage_>& other) {
   *this = internal::RotationDiffConversionTraits<AngleAxisDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived());
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RotationDiffConversionTraits<OtherDerived_, AngleAxisDiff, RotationDerived_>::convert(rotation.derived(), *this);
  }



  /*! \brief Reading access to the time derivative of the rotation angle.
   *  \returns rotation angle (scalar) with reading access
   */
  inline Scalar angle() const {
    return angle_;
  }

  /*! \brief Writing access to the time derivative of the rotation angle.
   *  \returns rotation angle (scalar) with writing access
   */
  inline Scalar& angle() {
    return angle_;
  }

  /*! \brief Reading access to the time derivative of the rotation axis.
   *  \returns rotation axis (vector) with reading access
   */
  inline const Vector3& axis() const {
    return axis_;
  }

  /*! \brief Writing access to the time derivative of the rotation axis.
   *  Attention: No length check in debug mode.
   *  \returns rotation axis (vector) with writing access
   */
  inline Vector3& axis() {
    return axis_;
  }

  /*! \returns the angle and axis in a 4x1 vector [angle; axis].
   */
  inline Vector4 vector() const {
    Vector4 vector;
    vector(0) = angle();
    vector.template block<3,1>(1,0) = axis();
    return vector;
  }

  /*! \brief Sets all time derivatives to zero.
   *  \returns reference
   */
  AngleAxisDiff& setZero() {
    angle_ = Scalar(0.0);
    axis_.setZero();
    return *this;
  }


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const AngleAxisDiff& diff) {
    out << diff.angle() << ", " << diff.axis().transpose();
    return out;
  }

 private:
  Scalar angle_;
  Vector3 axis_;
};


//! \brief Time derivative of a rotation quaternion with primitive type double
typedef AngleAxisDiff<double, RotationUsage::PASSIVE> AngleAxisDiffPD;
//! \brief Time derivative of a rotation quaternion with primitive type float
typedef AngleAxisDiff<float, RotationUsage::PASSIVE> AngleAxisDiffPF;
//! \brief Time derivative of a rotation quaternion with primitive type double
typedef AngleAxisDiff<double, RotationUsage::ACTIVE> AngleAxisDiffAD;
//! \brief Time derivative of a rotation quaternion with primitive type float
typedef AngleAxisDiff<float, RotationUsage::ACTIVE> AngleAxisDiffAF;

} // namespace eigen_impl

namespace internal {


template<typename PrimType_, enum RotationUsage Usage_>
class RotationDiffConversionTraits<eigen_impl::AngleAxisDiff<PrimType_, Usage_>, eigen_impl::LocalAngularVelocity<PrimType_, Usage_>, eigen_impl::AngleAxis<PrimType_, Usage_>> {
 public:
  inline static eigen_impl::AngleAxisDiff<PrimType_, Usage_> convert(const eigen_impl::AngleAxis<PrimType_, Usage_>& angleAxis, const eigen_impl::LocalAngularVelocity<PrimType_, Usage_>& angularVelocity) {
    typedef typename eigen_impl::AngleAxis<PrimType_, Usage_>::Vector3 Vector;
    const PrimType_ angle = angleAxis.angle();


    if (angle < 1e-14) {
      KINDR_ASSERT_TRUE(std::runtime_error, false, "not correctly implemented!");
      const PrimType_ angleDiff = angleAxis.axis().transpose()*angularVelocity.toImplementation();

      Vector axisDiff = 0.5*linear_algebra::getSkewMatrixFromVector(angleAxis.axis())*angularVelocity.toImplementation();
      return eigen_impl::AngleAxisDiff<PrimType_, Usage_>(angleDiff, axisDiff);
    }
    const Vector axis = angleAxis.axis();
    const Eigen::Matrix<PrimType_, 3, 3> n_hat = linear_algebra::getSkewMatrixFromVector(axis);
    const PrimType_ angleDiff = angleAxis.axis().transpose()*angularVelocity.toImplementation();
    const Vector axisDiff = (-0.5*sin(angle)/(1.0-cos(angle))*n_hat*n_hat+0.5*n_hat)*angularVelocity.toImplementation();
    return eigen_impl::AngleAxisDiff<PrimType_, Usage_>(angleDiff, axisDiff);
  }
};


} // namespace internal
} // namespace rotations
} // namespace kindr




#endif /* KINDR_ROTATIONS_EIGEN_ANGLEAXIS_DIFF_HPP_ */
