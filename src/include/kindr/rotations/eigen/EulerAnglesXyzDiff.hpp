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
#ifndef KINDR_ROTATIONS_EIGEN_EULERANGLESXYZ_DIFF_HPP_
#define KINDR_ROTATIONS_EIGEN_EULERANGLESXYZ_DIFF_HPP_

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

/*! \class EulerAnglesXyzDiff
 * \brief Implementation of time derivatives of Euler angles (X,Y',Z'' / roll,pitch,yaw) based on Eigen::Matrix<Scalar, 3, 1>
 *
 * The following two typedefs are provided for convenience:
 *   - EulerAnglesXyzDiffAD for primitive type double
 *   - EulerAnglesXyzDiffAF for primitive type float
 * \tparam PrimType_ the primitive type of the data (double or float)
 * \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_ >
class EulerAnglesXyzDiff : public EulerAnglesDiffXyzBase<EulerAnglesXyzDiff<PrimType_,Usage_>,Usage_> {
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
  inline explicit EulerAnglesXyzDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RotationDiffBase<OtherDerived_, Usage_>& other)
    : xyzDiff_(internal::RotationDiffConversionTraits<EulerAnglesXyzDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toImplementation()){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
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


   using EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_,Usage_>,Usage_>::operator+; // otherwise ambiguous EulerAnglesDiffBase and Eigen


   using EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_,Usage_>,Usage_>::operator-; // otherwise ambiguous EulerAnglesDiffBase and Eigen


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
typedef EulerAnglesXyzDiff<double, RotationUsage::PASSIVE> EulerAnglesXyzDiffPD;
//! \brief Time derivative of Euler angles with x-y-z convention and primitive type float
typedef EulerAnglesXyzDiff<float, RotationUsage::PASSIVE> EulerAnglesXyzDiffPF;
//! \brief Time derivative of Euler angles with x-y-z convention and primitive type double
typedef EulerAnglesXyzDiff<double, RotationUsage::ACTIVE> EulerAnglesXyzDiffAD;
//! \brief Time derivative of Euler angles with x-y-z convention and primitive type float
typedef EulerAnglesXyzDiff<float, RotationUsage::ACTIVE> EulerAnglesXyzDiffAF;


} // namespace eigen_impl

namespace internal {



template<typename PrimType_>
class RotationDiffConversionTraits<eigen_impl::EulerAnglesXyzDiff<PrimType_, RotationUsage::PASSIVE>, eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::PASSIVE>, eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE>> {
 public:
  inline static eigen_impl::EulerAnglesXyzDiff<PrimType_, RotationUsage::PASSIVE> convert(const eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE>& eulerAngles, const eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::PASSIVE>& angularVelocity) {
    const PrimType_ beta_Var = eulerAngles.pitch();
    const PrimType_ gamma_Var = eulerAngles.yaw();
    const PrimType_ w1 = angularVelocity.toImplementation()(0);
    const PrimType_ w2 = angularVelocity.toImplementation()(1);
    const PrimType_ w3 = angularVelocity.toImplementation()(2);
    const PrimType_ t2 = cos(beta_Var);
    KINDR_ASSERT_TRUE(std::runtime_error, t2 != PrimType_(0), "Error: cos(y) is zero! This case is not yet implemented!");
    const PrimType_ t3 = 1.0/t2;
    const PrimType_ t4 = cos(gamma_Var);
    const PrimType_ t5 = sin(gamma_Var);
    const PrimType_ t6 = sin(beta_Var);
    return eigen_impl::EulerAnglesXyzDiff<PrimType_, RotationUsage::PASSIVE>(t3*t4*w1-t3*t5*w2, t4*w2+t5*w1, w3-t3*t4*t6*w1+t3*t5*t6*w2);

  }
};


} // namespace internal
} // namespace rotations
} // namespace kindr




#endif /* KINDR_ROTATIONS_EIGEN_EULERANGLESXYZ_DIFF_HPP_ */
