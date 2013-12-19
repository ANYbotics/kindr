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
class AngleAxisDiff : public AngleAxisDiffBase<AngleAxisDiff<PrimType_, Usage_>,Usage_>, private Eigen::AngleAxis<PrimType_> {
 private:
  /*! \brief The base type.
   */
  typedef typename Eigen::AngleAxis<PrimType_> Base;

 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;

  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief The axis type is a 3D vector.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Vector3;

  /*! \brief Default constructor sets all derivatives to zero
   */
  AngleAxisDiff()
    : Base(Base::Zero()) {
  }

  explicit AngleAxisDiff(const Base& other) // explicit on purpose
    : Base(other) {
  }

  /*! \brief Constructor using four scalars.
   *  \param angle   time derivative of the rotation angle
   *  \param v1      first entry of the time derivative of the rotation axis vector
   *  \param v2      second entry of the time derivative of the rotation axis vector
   *  \param v3      third entry of the time derivative of the rotation axis vector
   */
  AngleAxisDiff(Scalar angle, Scalar v1, Scalar v2, Scalar v3)
    : Base(angle,Vector3(v1,v2,v3)) {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit AngleAxisDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : Base(internal::RDiffConversionTraits<AngleAxisDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived())){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RDiffConversionTraits<OtherDerived_, AngleAxisDiff, RotationDerived_>::convert(rotation.derived(), *this);
  }


  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return this->toImplementation();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return this->toImplementation();
  }

  /*! \brief Reading access to the time derivative of the rotation angle.
   *  \returns rotation angle (scalar) with reading access
   */
  inline Scalar angle() const {
    return Base::angle();
  }

  /*! \brief Writing access to the time derivative of the rotation angle.
   *  \returns rotation angle (scalar) with writing access
   */
  inline Scalar& angle() {
    return Base::angle();
  }

  /*! \brief Reading access to the time derivative of the rotation axis.
   *  \returns rotation axis (vector) with reading access
   */
  inline const Vector3& axis() const {
    return Base::axis();
  }

  /*! \brief Writing access to the time derivative of the rotation axis.
   *  Attention: No length check in debug mode.
   *  \returns rotation axis (vector) with writing access
   */
  inline Vector3& axis() {
    return Base::axis();
  }


  /*! \brief Sets all time derivatives to zero.
   *  \returns reference
   */
  AngleAxisDiff& setZero() {
    this->Base::setZero();
    return *this;
  }


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const AngleAxisDiff& diff) {
    out << diff.angle() << ", " << diff.axis().transpose();
    return out;
  }
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


template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::AngleAxisDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::AngleAxis<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::AngleAxisDiff<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::AngleAxis<PrimType_, RotationUsage::ACTIVE>& angleAxis, const eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>& angularVelocity) {
    typedef typename eigen_impl::AngleAxis<PrimType_, RotationUsage::ACTIVE>::Vector3 Vector;
    const PrimType_ angle = angleAxis.angle();


    if (angle < 1e-14) {
      const PrimType_ angleDiff = angularVelocity.toImplementation().norm();
      const Vector axisDiff = angularVelocity.toImplementation().normalized();
      return eigen_impl::AngleAxisDiff<PrimType_, RotationUsage::ACTIVE>(angleDiff, axisDiff);
    }
    const Vector axis = angleAxis.axis();
    const Eigen::Matrix<PrimType_, 3, 3> n_hat = linear_algebra::getSkewMatrixFromVector(axis);
    const PrimType_ angleDiff = angleAxis.axis().transpose()*angularVelocity;
    const Vector axisDiff = (-0.5*sin(angle)/(1.0-cos(angle))*n_hat-0.5)*n_hat*angularVelocity.toImplementation();
    return eigen_impl::AngleAxisDiff<PrimType_, RotationUsage::ACTIVE>(angleDiff, axisDiff);
  }
};


} // namespace internal
} // namespace rotations
} // namespace kindr




#endif /* KINDR_ROTATIONS_EIGEN_ANGLEAXIS_DIFF_HPP_ */
