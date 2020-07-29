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

#pragma once

#include <Eigen/Core>

#include "kindr/math/LinearAlgebra.hpp"
#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationDiffBase.hpp"
#include "kindr/rotations/Rotation.hpp"
#include "kindr/quaternions/Quaternion.hpp"
#include "kindr/phys_quant/PhysicalQuantities.hpp"

namespace kindr {

/*! \class LocalAngularVelocity
 * \brief Angular velocity in 3D-space expressed in local coordinates (frame fixed to the body).
 *
 * This class implements an angular velocity of a rigid body in 3D-space expressed in body fixed (local) frame.
 *
 * The angular velocity should represent the absolute rotational velocity of a rigid body with respect to an inertial (global) frame I
 * and its coordinates are expressed in the body fixed (local) frame (\f$\B_Omega = B_\omega_{IB}$\f)
 *
 * \tparam PrimType_  Primitive type of the coordinates.
 * \ingroup rotations
 */
template<typename PrimType_>
class LocalAngularVelocity : public RotationDiffBase<LocalAngularVelocity<PrimType_>>, public AngularVelocity<PrimType_, 3> {
 private:
  /*! \brief The base type.
   */
  typedef AngularVelocity<PrimType_, 3> Base;
 public:
  /*! \brief The implementation type.
   *
   *  The implementation type is always an Eigen object.
   */
  typedef typename Base::Implementation Implementation;

  /*! \brief The primitive type of the velocities.
   */
  typedef PrimType_ Scalar;

  /*! \brief Default constructor initializes all velocities with zero.
   */
  LocalAngularVelocity()
    : Base(Base::Zero()) {
  }

  /*! Constructor with three components (x,y,z)
   * \param x   x-coordinate expressed in body fixed (local) frame
   * \param y   y-coordinate expressed in body fixed (local) frame
   * \param z   z-coordinate expressed in body fixed (local) frame
   */
  LocalAngularVelocity(Scalar x, Scalar y, Scalar z)
    : Base(x, y, z) {
  }

  /*! \brief Constructor using vector class.
   *  \param other   AngularVelocity<PrimType_, 3>
   */
  explicit LocalAngularVelocity(const Base& other)
    : Base(other) {
   }

  /*! Constructor with three components (x,y,z)
   * \param vector  Eigen::Matrix<Scalar,3,1>
   */
  LocalAngularVelocity(const Implementation & vector)
    : Base(vector) {
  }

  /*! Constructor with a time derivative of a rotation with a different parameterization
   *
   * \param rotation    rotation the time derivative is taken at
   * \param other       other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit LocalAngularVelocity(const RotationBase<RotationDerived_>& rotation, const RotationDiffBase<OtherDerived_>& other)
    : Base(internal::RotationDiffConversionTraits<LocalAngularVelocity, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived())){
  }

  inline Implementation vector() const {
    return Base::toImplementation();
  }

  inline Implementation& vector() {
    return Base::toImplementation();
  }


  inline Implementation& toImplementation() {
    return Base::toImplementation();
  }

  inline const Implementation& toImplementation() const {
    return Base::toImplementation();
  }

  /*! \brief Cast to the base type.
   *  \returns the base (recommended only for advanced users)
   */
  inline Base& toBase() {
    return static_cast<Base&>(*this);
  }

  /*! \brief Cast to the base type.
   *  \returns the base (recommended only for advanced users)
   */
  inline const Base& toBase() const {
    return static_cast<const Base&>(*this);
  }

  /*! \brief Addition of two angular velocities.
   */
  template<typename Other_>
  LocalAngularVelocity operator +(const Other_& other) const {
    return LocalAngularVelocity(this->toBase() + other.toBase());
  }

  /*! \brief Subtraction of two angular velocities.
   */
  template<typename Other_>
  LocalAngularVelocity operator -(const Other_& other) const {
    return LocalAngularVelocity(this->toBase() - other.toBase());
  }

  /*! \brief Multiplication of an angular velocity with a scalar.
   */
  template<typename PrimTypeFactor_>
  LocalAngularVelocity operator *(const PrimTypeFactor_& factor) const {
    return LocalAngularVelocity(this->toBase()*factor);
  }

  /*! \brief Addition of two angular velocities.
   * \param other   other angular velocity
   */
  template<typename Other_>
  LocalAngularVelocity& operator +=(const Other_& other) {
    this->toBase() += other.toBase();
    return *this;
  }

  /*! \brief Subtraction of two angular velocities.
   * \param other   other angular velocity
   */
  template<typename Other_>
  LocalAngularVelocity& operator -=(const Other_& other) {
    this->toBase() -= other.toBase();
    return *this;
  }

  /*! \brief Sets all components of the angular velocity to zero.
   * \returns reference
   */
  LocalAngularVelocity& setZero() {
    Base::setZero();
    return *this;
  }

  /*! \brief Get zero element.
   * \returns zero element
   */
  static LocalAngularVelocity Zero() {
    return LocalAngularVelocity(Implementation::Zero());
  }
};

/*! \brief Multiplication of an angular velocity with a scalar.
 */
template<typename PrimType_, typename PrimTypeFactor_>
LocalAngularVelocity<PrimType_> operator *(PrimTypeFactor_ factor, const LocalAngularVelocity<PrimType_>& localAngularVelocity) {
  return LocalAngularVelocity<PrimType_>(localAngularVelocity.toBase()*factor);
}

//! \brief 3D angular velocity with primitive type double
typedef LocalAngularVelocity<double>  LocalAngularVelocityPD;
//! \brief 3D angular velocity with primitive type float
typedef LocalAngularVelocity<float>  LocalAngularVelocityPF;
//! \brief 3D angular velocity with primitive type double
typedef LocalAngularVelocity<double>  LocalAngularVelocityD;
//! \brief 3D angular velocity with primitive type float
typedef LocalAngularVelocity<float>  LocalAngularVelocityF;



namespace internal {

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename PrimType_, typename Rotation_>
class RotationDiffConversionTraits<LocalAngularVelocity<PrimType_>, GlobalAngularVelocity<PrimType_>, Rotation_> {
 public:
  inline static LocalAngularVelocity<PrimType_> convert(const Rotation_& rotationLocalToGlobal, const GlobalAngularVelocity<PrimType_>& globalAngularVelocity) {
    return LocalAngularVelocity<PrimType_>(rotationLocalToGlobal.inverseRotate(globalAngularVelocity.toImplementation()));
  }
};

template<typename PrimType_>
class RotationDiffConversionTraits<LocalAngularVelocity<PrimType_>, RotationQuaternionDiff<PrimType_>, RotationQuaternion<PrimType_>> {
 public:
  inline static LocalAngularVelocity<PrimType_> convert(const RotationQuaternion<PrimType_>& rquat, const RotationQuaternionDiff<PrimType_>& rquatdiff) {
    return LocalAngularVelocity<PrimType_>(2.0*rquat.getLocalQuaternionDiffMatrix()*rquatdiff.toQuaternion().vector());
  }
};

//! \f$B_\hat{w}_IB = (R_IB*dR_IB')$\f
template<typename PrimType_>
class RotationDiffConversionTraits<LocalAngularVelocity<PrimType_>, RotationMatrixDiff<PrimType_>, RotationMatrix<PrimType_>> {
 public:
  inline static LocalAngularVelocity<PrimType_> convert(const RotationMatrix<PrimType_>& rotationMatrix, const RotationMatrixDiff<PrimType_>& rotationMatrixDiff) {
    return LocalAngularVelocity<PrimType_>(getVectorFromSkewMatrix<PrimType_>(rotationMatrix.inverted().matrix()*rotationMatrixDiff.matrix()));
  }
};



template<typename PrimType_>
class RotationDiffConversionTraits<LocalAngularVelocity<PrimType_>, EulerAnglesZyxDiff<PrimType_>, EulerAnglesZyx<PrimType_>> {
 public:
  inline static LocalAngularVelocity<PrimType_> convert(const EulerAnglesZyx<PrimType_>& eulerAngles, const EulerAnglesZyxDiff<PrimType_>& eulerAnglesDiff) {
    return LocalAngularVelocity<PrimType_>(eulerAngles.getMappingFromDiffToLocalAngularVelocity()*eulerAnglesDiff.toImplementation());

  }
};

template<typename PrimType_>
class RotationDiffConversionTraits<LocalAngularVelocity<PrimType_>, EulerAnglesXyzDiff<PrimType_>, EulerAnglesXyz<PrimType_>> {
 public:
  inline static LocalAngularVelocity<PrimType_> convert(const EulerAnglesXyz<PrimType_>& eulerAngles, const EulerAnglesXyzDiff<PrimType_>& eulerAnglesDiff) {
    return LocalAngularVelocity<PrimType_>(eulerAngles.getMappingFromDiffToLocalAngularVelocity()*eulerAnglesDiff.toImplementation());


  }
};

} // namespace internal
} // namespace kindr

