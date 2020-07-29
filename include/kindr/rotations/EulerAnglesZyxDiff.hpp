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

/*! \class EulerAnglesZyxDiff
 * \brief Implementation of time derivatives of Euler angles (Z,Y',X'' / yaw,pitch,roll) based on Eigen::Matrix<Scalar, 3, 1>
 *
 * The following two typedefs are provided for convenience:
 *   - EulerAnglesZyxDiffAD for primitive type double
 *   - EulerAnglesZyxDoffAF for primitive type float
 * \tparam PrimType_ the primitive type of the data (double or float)
 * \ingroup rotations
 */
template<typename PrimType_>
class EulerAnglesZyxDiff : public RotationDiffBase<EulerAnglesZyxDiff<PrimType_>> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

  /*! \brief data container [yaw; pitch; roll]
   */
  Base zyxDiff_;

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
  EulerAnglesZyxDiff()
    : zyxDiff_(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param yaw      time derivative of first rotation angle around Z axis
   *  \param pitch    time derivative of second rotation angle around Y' axis
   *  \param roll     time derivative of third rotation angle around X'' axis
   */
  EulerAnglesZyxDiff(Scalar yaw, Scalar pitch, Scalar roll)
    : zyxDiff_(yaw,pitch,roll) {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit EulerAnglesZyxDiff(const RotationBase<RotationDerived_>& rotation, const RotationDiffBase<OtherDerived_>& other)
    : zyxDiff_(internal::RotationDiffConversionTraits<EulerAnglesZyxDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toImplementation()){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_>& rotation) const {
    return internal::RotationDiffConversionTraits<OtherDerived_, EulerAnglesZyxDiff, RotationDerived_>::convert(rotation.derived(), *this);
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<Scalar, 3, 1> [yaw; pitch; roll]
   */
  explicit EulerAnglesZyxDiff(const Base& other)
    : zyxDiff_(other) {
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Base& toImplementation() {
    return static_cast<Base&>(zyxDiff_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base& toImplementation() const {
    return static_cast<const Base&>(zyxDiff_);
  }


  inline Base& vector() {
    return toImplementation();
  }

  inline const Base& vector() const {
    return toImplementation();
  }


  /*! \brief Reading access to time derivative of yaw (Z) angle.
    *  \returns time derivative of yaw angle (scalar) with reading access
    */
   inline Scalar yaw() const {
     return toImplementation()(0);
   }

   /*! \brief Reading access to time derivative of pitch (Y') angle.
    *  \returns time derivative of pitch angle (scalar) with reading access
    */
   inline Scalar pitch() const {
     return toImplementation()(1);
   }

   /*! \brief Reading access to time derivative of roll (X'') angle.
    *  \returns time derivative of roll angle (scalar) with reading access
    */
   inline Scalar roll() const {
     return toImplementation()(2);
   }

   /*! \brief Writing access to time derivative of yaw (Z) angle.
    *  \returns time derivative of yaw angle (scalar) with writing access
    */
   inline Scalar& yaw() {
     return toImplementation()(0);
   }

   /*! \brief Writing access to time derivative of pitch (Y') angle.
    *  \returns time derivative of pitch angle (scalar) with writing access
    */
   inline Scalar& pitch() {
     return toImplementation()(1);
   }

   /*! \brief Writing access to time derivative of roll (X'') angle.
    *  \returns time derivative of roll angle (scalar) with writing access
    */
   inline Scalar& roll() {
     return toImplementation()(2);
   }

   /*! \brief Reading access to time derivative of yaw (Z) angle.
    *  \returns time derivative of yaw angle (scalar) with reading access
    */
   inline Scalar z() const {
     return toImplementation()(0);
   }

   /*! \brief Reading access to time derivative of pitch (Y') angle.
    *  \returns time derivative of pitch angle (scalar) with reading access
    */
   inline Scalar y() const {
     return toImplementation()(1);
   }

   /*! \brief Reading access to time derivative of roll (X'') angle.
    *  \returns time derivative of roll angle (scalar) with reading access
    */
   inline Scalar x() const {
     return toImplementation()(2);
   }

   /*! \brief Writing access to time derivative of yaw (Z) angle.
    *  \returns time derivative of yaw angle (scalar) with writing access
    */
   inline Scalar& z() {
     return toImplementation()(0);
   }

   /*! \brief Writing access to time derivative of pitch (Y') angle.
    *  \returns time derivative of pitch angle (scalar) with writing access
    */
   inline Scalar& y() {
     return toImplementation()(1);
   }

   /*! \brief Writing access to time derivative of roll (X'') angle.
    *  \returns time derivative of roll angle (scalar) with writing access
    */
   inline Scalar& x() {
     return toImplementation()(2);
   }

   /*! \brief Sets all time derivatives to zero.
    *  \returns reference
    */
   EulerAnglesZyxDiff& setZero() {
     this->toImplementation().setZero();
     return *this;
   }

   /*! \brief Get zero element.
    *  \returns zero element
    */
   static EulerAnglesZyxDiff Zero() {
     return EulerAnglesZyxDiff(Base::Zero());
   }

   /*! \brief Addition of two angular velocities.
    */
   using RotationDiffBase<EulerAnglesZyxDiff<PrimType_>>::operator+; // otherwise ambiguous EulerAnglesDiffBase and Eigen

   /*! \brief Subtraction of two angular velocities.
    */
   using RotationDiffBase<EulerAnglesZyxDiff<PrimType_>>::operator-; // otherwise ambiguous EulerAnglesDiffBase and Eigen


   /*! \brief Used for printing the object with std::cout.
    *
    *   Prints: yaw pitch roll
    *  \returns std::stream object
    */
   friend std::ostream& operator << (std::ostream& out, const EulerAnglesZyxDiff& diff) {
     out << diff.toImplementation().transpose();
     return out;
   }
};

//! \brief Time derivative of Euler angles with z-y-x convention and primitive type double
typedef EulerAnglesZyxDiff<double> EulerAnglesZyxDiffPD;
//! \brief Time derivative of Euler angles with z-y-x convention and primitive type float
typedef EulerAnglesZyxDiff<float> EulerAnglesZyxDiffPF;
//! \brief Time derivative of Euler angles with z-y-x convention and primitive type double
typedef EulerAnglesZyxDiff<double> EulerAnglesZyxDiffD;
//! \brief Time derivative of Euler angles with z-y-x convention and primitive type float
typedef EulerAnglesZyxDiff<float> EulerAnglesZyxDiffF;



namespace internal {


template<typename PrimType_>
class RotationDiffConversionTraits<EulerAnglesZyxDiff<PrimType_>, LocalAngularVelocity<PrimType_>, EulerAnglesZyx<PrimType_>> {
 public:
  inline static EulerAnglesZyxDiff<PrimType_> convert(const EulerAnglesZyx<PrimType_>& eulerAngles, const LocalAngularVelocity<PrimType_>& angularVelocity) {
    return EulerAnglesZyxDiff<PrimType_>(eulerAngles.getMappingFromLocalAngularVelocityToDiff()*angularVelocity.vector());
  }
};


} // namespace internal
} // namespace kindr

