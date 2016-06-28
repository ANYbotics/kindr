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

namespace kindr {

/*! \class EulerAnglesXyzDiff
 * \brief Implementation of time derivatives of Euler angles (X,Y',Z'' / roll,pitch,yaw) based on Eigen::Matrix<Scalar, 3, 1>
 *
 * The following two typedefs are provided for convenience:
 *   - EulerAnglesXyzDiffAD for primitive type double
 *   - EulerAnglesXyzDiffAF for primitive type float
 * \tparam PrimType_ the primitive type of the data (double or float)
 * \ingroup rotations
 */
template<typename PrimType_>
class EulerAnglesXyzDiff : public RotationDiffBase<EulerAnglesXyzDiff<PrimType_>> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

  /*! \brief data container [roll; pitch; yaw]
   */
  Base xyzDiff_;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief Default constructor.
   */
  EulerAnglesXyzDiff()
    : xyzDiff_(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param roll     time derivative of first rotation angle around X axis
   *  \param pitch    time derivative of second rotation angle around Y' axis
   *  \param yaw      time derivative of third rotation angle around Z'' axis
   */
  EulerAnglesXyzDiff(Scalar roll, Scalar pitch, Scalar yaw)
    : xyzDiff_(roll,pitch,yaw) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,3,1> [roll; pitch; yaw]
   */
  explicit EulerAnglesXyzDiff(const Base& other)
    : xyzDiff_(other) {
  }


  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit EulerAnglesXyzDiff(const RotationBase<RotationDerived_>& rotation, const RotationDiffBase<OtherDerived_>& other)
    : xyzDiff_(internal::RotationDiffConversionTraits<EulerAnglesXyzDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toImplementation()){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_>& rotation) const {
    return internal::RotationDiffConversionTraits<OtherDerived_, EulerAnglesXyzDiff, RotationDerived_>::convert(rotation.derived(), *this);
  }


  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Base& toImplementation() {
    return static_cast<Base&>(xyzDiff_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base& toImplementation() const {
    return static_cast<const Base&>(xyzDiff_);
  }

  inline Base& vector() {
    return toImplementation();
  }

  inline const Base& vector() const {
    return toImplementation();
  }


  /*! \brief Reading access to time derivative of roll (X) angle.
   *  \returns roll angle (scalar) with reading access
   */
  inline Scalar roll() const {
    return toImplementation()(0);
  }

  /*! \brief Reading access to time derivative of pitch (Y') angle.
   *  \returns pitch angle (scalar) with reading access
   */
  inline Scalar pitch() const {
    return toImplementation()(1);
  }

  /*! \brief Reading access to time derivative of yaw (Z'') angle.
   *  \returns yaw angle (scalar) with reading access
   */
  inline Scalar yaw() const {
    return toImplementation()(2);
  }

  /*! \brief Writing access to time derivative of roll (X) angle.
   *  \returns roll angle (scalar) with writing access
   */
  inline Scalar& roll() {
    return toImplementation()(0);
  }

  /*! \brief Writing access to time derivative of pitch (Y') angle.
   *  \returns pitch angle (scalar) with writing access
   */
  inline Scalar& pitch() {
    return toImplementation()(1);
  }

  /*! \brief Writing access to time derivative of yaw (Z'') angle.
   *  \returns yaw angle (scalar) with writing access
   */
  inline Scalar& yaw() {
    return toImplementation()(2);
  }

  /*! \brief Reading access to time derivative of roll (X) angle.
   *  \returns roll angle (scalar) with reading access
   */
  inline Scalar x() const {
    return toImplementation()(0);
  }

  /*! \brief Reading access to time derivative of pitch (Y') angle.
   *  \returns pitch angle (scalar) with reading access
   */
  inline Scalar y() const {
    return toImplementation()(1);
  }

  /*! \brief Reading access to time derivative of yaw (Z'') angle.
   *  \returns yaw angle (scalar) with reading access
   */
  inline Scalar z() const {
    return toImplementation()(2);
  }

  /*! \brief Writing access to time derivative of roll (X) angle.
   *  \returns roll angle (scalar) with writing access
   */
  inline Scalar& x() {
    return toImplementation()(0);
  }

  /*! \brief Writing access to time derivative of pitch (Y') angle.
   *  \returns pitch angle (scalar) with writing access
   */
  inline Scalar& y() {
    return toImplementation()(1);
  }

  /*! \brief Writing access to time derivative of yaw (Z'') angle.
   *  \returns yaw angle (scalar) with writing access
   */
  inline Scalar& z() {
    return toImplementation()(2);
  }

   /*! \brief Sets all time derivatives to zero.
    *  \returns reference
    */
   EulerAnglesXyzDiff& setZero() {
     this->toImplementation().setZero();
     return *this;
   }


   using RotationDiffBase<EulerAnglesXyzDiff<PrimType_>>::operator+; // otherwise ambiguous RotationDiffBase and Eigen

   using RotationDiffBase<EulerAnglesXyzDiff<PrimType_>>::operator-; // otherwise ambiguous RotationDiffBase and Eigen

   Eigen::Matrix<PrimType_, 3, 3> getMappingFromLocalAngularVelocityToSecondDiff(const EulerAnglesXyz<PrimType_>& rotation) const {
     using std::sin;
     using std::cos;
     Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
     const PrimType_ x = rotation.x();
     const PrimType_ y = rotation.y();
     const PrimType_ z = rotation.z();
     const PrimType_ dx = this->x();
     const PrimType_ dy = this->y();
     const PrimType_ dz = this->z();
     const PrimType_ t2 = cos(y);
     KINDR_ASSERT_TRUE(std::runtime_error, t2 != PrimType_(0), "Gimbal lock: cos(y) is zero!");
     const PrimType_ t3 = 1.0/t2;
     const PrimType_ t4 = cos(z);
     const PrimType_ t5 = 1.0/(t2*t2);
     const PrimType_ t6 = sin(y);
     const PrimType_ t7 = sin(z);
     const PrimType_ t8 = t6*t6;
     mat(0,0) = -dz*t3*t7+dy*t4*t5*t6;
     mat(0,1) = -dz*t3*t4-dy*t5*t6*t7;
     mat(1,0) = dz*t4;
     mat(1,1) = -dz*t7;
     mat(2,0) = -dy*(t4+t4*t5*t8)+dz*t3*t6*t7;
     mat(2,1) = dy*(t7+t5*t7*t8)+dz*t3*t4*t6;

     return mat;
   }

   Eigen::Matrix<PrimType_, 3, 3> getMappingFromSecondDiffToLocalAngularVelocity(const EulerAnglesXyz<PrimType_>& rotation) const {
     using std::sin;
     using std::cos;
     Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
     const PrimType_ x = rotation.x();
     const PrimType_ y = rotation.y();
     const PrimType_ z = rotation.z();
     const PrimType_ dx = this->x();
     const PrimType_ dy = this->y();
     const PrimType_ dz = this->z();
     const PrimType_ t2 = cos(z);
     const PrimType_ t3 = cos(y);
     const PrimType_ t4 = sin(y);
     const PrimType_ t5 = sin(z);
     mat(0,0) = -dy*t2*t4-dz*t3*t5;
     mat(0,1) = dz*t2;
     mat(1,0) = dy*t4*t5-dz*t2*t3;
     mat(1,1) = -dz*t5;
     mat(2,0) = dy*t3;
     return mat;
   }

   Eigen::Matrix<PrimType_, 3, 3> getMappingFromGlobalAngularVelocityToSecondDiff(const EulerAnglesXyz<PrimType_>& rotation) const {
     using std::sin;
     using std::cos;
     Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
     const PrimType_ x = rotation.x();
     const PrimType_ y = rotation.y();
     const PrimType_ z = rotation.z();
     const PrimType_ dx = this->x();
     const PrimType_ dy = this->y();
     const PrimType_ dz = this->z();
     const PrimType_ t2 = sin(x);
     const PrimType_ t3 = sin(y);
     const PrimType_ t4 = cos(y);
     const PrimType_ t5 = cos(x);
     KINDR_ASSERT_TRUE(std::runtime_error, t4 != PrimType_(0), "Gimbal lock: cos(y) is zero!");
     const PrimType_ t6 = 1.0/(t4*t4);
     const PrimType_ t7 = t3*t3;
     const PrimType_ t8 = 1.0/t4;
     mat(0,1) = dy*(t2+t2*t6*t7)+dx*t3*t5*t8;
     mat(0,2) = -dy*(t5+t5*t6*t7)+dx*t2*t3*t8;
     mat(1,1) = -dx*t2;
     mat(1,2) = dx*t5;
     mat(2,1) = -dx*t5*t8-dy*t2*t3*t6;
     mat(2,2) = -dx*t2*t8+dy*t3*t5*t6;

     return mat;
   }

   Eigen::Matrix<PrimType_, 3, 3> getMappingFromSecondDiffToGlobalAngularVelocity(const EulerAnglesXyz<PrimType_>& rotation) const {
     using std::sin;
     using std::cos;
     Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
     const PrimType_ x = rotation.x();
     const PrimType_ y = rotation.y();
     const PrimType_ z = rotation.z();
     const PrimType_ dx = this->x();
     const PrimType_ dy = this->y();
     const PrimType_ dz = this->z();
     const PrimType_ t2 = cos(y);
     const PrimType_ t3 = sin(x);
     const PrimType_ t4 = cos(x);
     const PrimType_ t5 = sin(y);
     mat(0, 2) = dy*t2;
     mat(1, 1) = -dx*t3;
     mat(1, 2) = -dx*t2*t4+dy*t3*t5;
     mat(2, 1) = dx*t4;
     mat(2, 2) = -dx*t2*t3-dy*t4*t5;
     return mat;
   }

   /*! \brief Used for printing the object with std::cout.
    *
    *   Prints: roll pitch yaw
    *  \returns std::stream object
    */
   friend std::ostream& operator << (std::ostream& out, const EulerAnglesXyzDiff& diff) {
     out << diff.toImplementation().transpose();
     return out;
   }
};

//! \brief Time derivative of Euler angles with x-y-z convention and primitive type double
typedef EulerAnglesXyzDiff<double> EulerAnglesXyzDiffPD;
//! \brief Time derivative of Euler angles with x-y-z convention and primitive type float
typedef EulerAnglesXyzDiff<float> EulerAnglesXyzDiffPF;
//! \brief Time derivative of Euler angles with x-y-z convention and primitive type double
typedef EulerAnglesXyzDiff<double> EulerAnglesXyzDiffD;
//! \brief Time derivative of Euler angles with x-y-z convention and primitive type float
typedef EulerAnglesXyzDiff<float> EulerAnglesXyzDiffF;



namespace internal {

template<typename PrimType_>
class RotationDiffConversionTraits<EulerAnglesXyzDiff<PrimType_>, LocalAngularVelocity<PrimType_>, EulerAnglesXyz<PrimType_>> {
 public:
  inline static EulerAnglesXyzDiff<PrimType_> convert(const EulerAnglesXyz<PrimType_>& eulerAngles, const LocalAngularVelocity<PrimType_>& angularVelocity) {
    return EulerAnglesXyzDiff<PrimType_>(eulerAngles.getMappingFromLocalAngularVelocityToDiff()*angularVelocity.vector());
  }
};


} // namespace internal
} // namespace kindr


