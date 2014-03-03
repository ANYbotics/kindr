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

#ifndef KINDR_ROTATIONS_EIGEN_EULERANGLESXYZ_HPP_
#define KINDR_ROTATIONS_EIGEN_EULERANGLESXYZ_HPP_

#include <cmath>

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationBase.hpp"
#include "kindr/rotations/eigen/RotationEigenFunctions.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {


/*! \class EulerAnglesXyz
 *  \brief Implementation of Euler angles (X-Y'-Z'' / roll-pitch-yaw) rotation based on Eigen::Matrix<Scalar,3,1>
 *
 *  The following typedefs are provided for convenience:
 *   - \ref eigen_impl::EulerAnglesXyzAD "EulerAnglesXyzAD" for active rotation and double primitive type
 *   - \ref eigen_impl::EulerAnglesXyzAF "EulerAnglesXyzAF" for active rotation and float primitive type
 *   - \ref eigen_impl::EulerAnglesXyzPD "EulerAnglesXyzPD" for passive rotation and double primitive type
 *   - \ref eigen_impl::EulerAnglesXyzPF "EulerAnglesXyzPF" for passive rotation and float primitive type
 *   - EulerAnglesRpyAD = EulerAnglesXyzAD
 *   - EulerAnglesRpyAF = EulerAnglesXyzAF
 *   - EulerAnglesRpyPD = EulerAnglesXyzPD
 *   - EulerAnglesRpyPF = EulerAnglesXyzPF
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *
 *  \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class EulerAnglesXyz : public EulerAnglesXyzBase<EulerAnglesXyz<PrimType_, Usage_>, Usage_> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

  /*! \brief vector of Euler angles [roll; pitch; yaw]
   */
  Base xyz_;

 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief Rotation Vector as 3x1-matrix
   */
  typedef Base Vector;

  /*! \brief Default constructor using identity rotation.
   */
  EulerAnglesXyz()
    : xyz_(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param roll     first rotation angle around X axis
   *  \param pitch    second rotation angle around Y' axis
   *  \param yaw      third rotation angle around Z'' axis
   */
  EulerAnglesXyz(Scalar roll, Scalar pitch, Scalar yaw)
    : xyz_(roll,pitch,yaw) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,3,1> [roll; pitch; yaw]
   */
  explicit EulerAnglesXyz(const Base& other)
  : xyz_(other) {
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit EulerAnglesXyz(const RotationBase<OtherDerived_, Usage_>& other)
    : xyz_(internal::ConversionTraits<EulerAnglesXyz, OtherDerived_>::convert(other.derived()).toImplementation()) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OtherDerived_>
  EulerAnglesXyz& operator =(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toImplementation() = internal::ConversionTraits<EulerAnglesXyz, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  EulerAnglesXyz& operator ()(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toImplementation() = internal::ConversionTraits<EulerAnglesXyz, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  EulerAnglesXyz inverted() const {
    return EulerAnglesXyz(eigen_internal::getInverseRpy<PrimType_, PrimType_>(this->toImplementation()));
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  EulerAnglesXyz& invert() {
    *this = this->inverted();
    return *this;
  }

  /*! \brief Returns the Euler angles in a vector.
   *  \returns  vector Eigen::Matrix<Scalar,3, 1>
   */
  inline const Vector vector() const {
    return this->toImplementation();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Base& toImplementation() {
    return static_cast<Base&>(xyz_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base& toImplementation() const {
    return static_cast<const Base&>(xyz_);
  }

  /*! \brief Returns roll (X) angle.
   *  \returns roll angle (scalar)
   */
  inline Scalar roll() const {
    return xyz_(0);
  }

  /*! \brief Returns pitch (Y') angle.
   *  \returns pitch angle (scalar)
   */
  inline Scalar pitch() const {
    return xyz_(1);
  }

  /*! \brief Returns yaw (Z'') angle.
   *  \returns yaw angle (scalar)
   */
  inline Scalar yaw() const {
    return xyz_(2);
  }

  /*! \brief Sets roll (X) angle.
   */
  inline void setRoll(Scalar roll) {
    xyz_(0) = roll;
  }

  /*! \brief Sets pitch (Y') angle.
   */
  inline void setPitch(Scalar pitch) {
    xyz_(1) = pitch;
  }

  /*! \brief Sets yaw (Z'') angle.
   */
  inline void setYaw(Scalar yaw) {
    xyz_(2) = yaw;
  }

  /*! \brief Gets roll (X) angle.
   *  \returns roll angle (scalar)
   */
  inline Scalar x() const {
    return xyz_(0);
  }

  /*! \brief Gets pitch (Y') angle.
   *  \returns pitch angle (scalar)
   */
  inline Scalar y() const {
    return xyz_(1);
  }

  /*! \brief Gets yaw (Z'') angle.
   *  \returns yaw angle (scalar)
   */
  inline Scalar z() const {
    return xyz_(2);
  }

  /*! \brief Sets roll (X) angle.
   */
  inline void setX(Scalar x) {
    xyz_(0) = x;
  }

  /*! \brief Sets pitch (Y') angle.
   */
  inline void setY(Scalar y) {
    xyz_(1) = y;
  }

  /*! \brief Sets yaw (Z'') angle.
   */
  inline void setZ(Scalar z) {
    xyz_(2) = z;
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  EulerAnglesXyz& setIdentity() {
    xyz_.setZero();
    return *this;
  }

  /*! \brief Returns a unique Euler angles rotation with angles in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  This function is used to compare different rotations.
   *  \returns copy of the Euler angles rotation which is unique
   */
  EulerAnglesXyz getUnique() const {
    Base xyz(kindr::common::floatingPointModulo(x()+M_PI,2*M_PI)-M_PI,
             kindr::common::floatingPointModulo(y()+M_PI,2*M_PI)-M_PI,
             kindr::common::floatingPointModulo(z()+M_PI,2*M_PI)-M_PI); // wrap all angles into [-pi,pi)

    const double tol = 1e-3;

    // wrap angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    if(xyz.y() < -M_PI/2 - tol)
    {
      if(xyz.x() < 0) {
        xyz.x() = xyz.x() + M_PI;
      } else {
        xyz.x() = xyz.x() - M_PI;
      }

      xyz.y() = -(xyz.y() + M_PI);

      if(xyz.z() < 0) {
        xyz.z() = xyz.z() + M_PI;
      } else {
        xyz.z() = xyz.z() - M_PI;
      }
    }
    else if(-M_PI/2 - tol <= xyz.y() && xyz.y() <= -M_PI/2 + tol)
    {
      xyz.x() -= xyz.z();
      xyz.z() = 0;
    }
    else if(-M_PI/2 + tol < xyz.y() && xyz.y() < M_PI/2 - tol)
    {
      // ok
    }
    else if(M_PI/2 - tol <= xyz.y() && xyz.y() <= M_PI/2 + tol)
    {
      // todo: M_PI/2 should not be in range, other formula?
      xyz.x() += xyz.z();
      xyz.z() = 0;
    }
    else // M_PI/2 + tol < xyz.y()
    {
      if(xyz.x() < 0) {
        xyz.x() = xyz.x() + M_PI;
      } else {
        xyz.x() = xyz.x() - M_PI;
      }

      xyz.y() = -(xyz.y() - M_PI);

      if(xyz.z() < 0) {
        xyz.z() = xyz.z() + M_PI;
      } else {
        xyz.z() = xyz.z() - M_PI;
      }
    }

    return EulerAnglesXyz(xyz);
  }

  /*! \brief Modifies the Euler angles rotation such that the angles lie in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  \returns reference
   */
  EulerAnglesXyz& setUnique() {
    *this = getUnique();
    return *this;
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using EulerAnglesXyzBase<EulerAnglesXyz<PrimType_, Usage_>, Usage_>::operator*;

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator==.
   *  \returns true if two rotations are similar.
   */
  using EulerAnglesXyzBase<EulerAnglesXyz<PrimType_, Usage_>, Usage_>::operator==;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const EulerAnglesXyz& xyz) {
    out << xyz.toImplementation().transpose();
    return out;
  }
};

//! \brief Active Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primitive type
typedef EulerAnglesXyz<double, RotationUsage::ACTIVE>  EulerAnglesXyzAD;
//! \brief Active Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primitive type
typedef EulerAnglesXyz<float,  RotationUsage::ACTIVE>  EulerAnglesXyzAF;
//! \brief Passive Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primitive type
typedef EulerAnglesXyz<double, RotationUsage::PASSIVE> EulerAnglesXyzPD;
//! \brief Passive Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primitive type
typedef EulerAnglesXyz<float,  RotationUsage::PASSIVE> EulerAnglesXyzPF;

//! \brief Equivalent Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) class
template <typename PrimType_, enum RotationUsage Usage_>
using EulerAnglesRpy = EulerAnglesXyz<PrimType_, Usage_>;

//! \brief Active Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primitive type
typedef EulerAnglesRpy<double, RotationUsage::ACTIVE>  EulerAnglesRpyAD;
//! \brief Active Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primitive type
typedef EulerAnglesRpy<float,  RotationUsage::ACTIVE>  EulerAnglesRpyAF;
//! \brief Passive Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primitive type
typedef EulerAnglesRpy<double, RotationUsage::PASSIVE> EulerAnglesRpyPD;
//! \brief Passive Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primitive type
typedef EulerAnglesRpy<float,  RotationUsage::PASSIVE> EulerAnglesRpyPF;


} // namespace eigen_impl


namespace internal {

template<typename PrimType_, enum RotationUsage Usage_>
class get_scalar<eigen_impl::EulerAnglesXyz<PrimType_, Usage_>> {
 public:
  typedef PrimType_ Scalar;
};

template<typename PrimType_, enum RotationUsage Usage_>
class get_matrix3X<eigen_impl::EulerAnglesXyz<PrimType_, Usage_>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE> OtherUsage;
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>, eigen_impl::AngleAxis<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_> convert(const eigen_impl::AngleAxis<SourcePrimType_, Usage_>& aa) {
//    return eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>(eigen_impl::getRpyFromAngleAxis<SourcePrimType_, DestPrimType_>(aa.toImplementation()));
    return eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>(eigen_impl::RotationQuaternion<DestPrimType_, Usage_>(aa));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>, eigen_impl::RotationVector<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_> convert(const eigen_impl::RotationVector<SourcePrimType_, Usage_>& rotationVector) {
    return eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>(eigen_impl::AngleAxis<SourcePrimType_, Usage_>(rotationVector));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>, eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>& q) {
    return eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>(eigen_impl::eigen_internal::getRpyFromQuaternion<SourcePrimType_, DestPrimType_>(q.toImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>, eigen_impl::RotationMatrix<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_> convert(const eigen_impl::RotationMatrix<SourcePrimType_, Usage_>& R) {
    return eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>(eigen_impl::eigen_internal::getRpyFromRotationMatrix<SourcePrimType_, DestPrimType_>(R.toImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>, eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>& xyz) {
    return eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>(xyz.toImplementation().template cast<DestPrimType_>());
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>, eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>& zyx) {
    return eigen_impl::EulerAnglesXyz<DestPrimType_, Usage_>(eigen_impl::eigen_internal::getRpyFromYpr<SourcePrimType_, DestPrimType_>(zyx.toImplementation()));
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Multiplication Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

//template<typename PrimType_>
//class MultiplicationTraits<RotationBase<eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE>, RotationUsage::ACTIVE>, RotationBase<eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE>, RotationUsage::ACTIVE>> {
// public:
//  inline static eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE> mult(const eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE>& a, const eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE>& b) {
//    return eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE>(eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>(
//                                                                 eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>(a).toImplementation()*
//                                                                 eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>(b).toImplementation()));
//  }
//};
//
//template<typename PrimType_>
//class MultiplicationTraits<RotationBase<eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE>, RotationUsage::PASSIVE>, RotationBase<eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE>, RotationUsage::PASSIVE>> {
// public:
//  inline static eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE> mult(const eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE>& a, const eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE>& b) {
//    return eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::PASSIVE>(eigen_impl::RotationQuaternion<PrimType_, RotationUsage::PASSIVE>(
//                                                                 eigen_impl::RotationQuaternion<PrimType_, RotationUsage::PASSIVE>(a).toImplementation()*
//                                                                 eigen_impl::RotationQuaternion<PrimType_, RotationUsage::PASSIVE>(b).toImplementation()));
//  }
//};

//template<typename PrimType_, enum RotationUsage Usage_>
//class MultiplicationTraits<RotationBase<eigen_impl::EulerAnglesXyz<PrimType_, Usage_>, Usage_>, RotationBase<eigen_impl::EulerAnglesXyz<PrimType_, Usage_>, Usage_>> {
// public:
//  inline static eigen_impl::EulerAnglesXyz<PrimType_, Usage_> mult(const eigen_impl::EulerAnglesXyz<PrimType_, Usage_>& a, const eigen_impl::EulerAnglesXyz<PrimType_, Usage_>& b) {
//    return eigen_impl::EulerAnglesXyz<PrimType_, Usage_>(eigen_impl::RotationQuaternion<PrimType_, Usage_>(
//                                                                 eigen_impl::RotationQuaternion<PrimType_, Usage_>(a).toImplementation()*
//                                                                 eigen_impl::RotationQuaternion<PrimType_, Usage_>(b).toImplementation()));
//  }
//};
//

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Comparison Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */



} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_EIGEN_EULERANGLESXYZ_HPP_ */
