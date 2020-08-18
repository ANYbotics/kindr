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

#include <cmath>

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationBase.hpp"

namespace kindr {

/*! \class EulerAnglesZyx
 *  \brief Implementation of Euler angles (intrinsic Z-Y'-X'', a.k.a. extrinsic x-y-z, a.k.a. yaw-pitch-roll angles) rotation.
 *
 *  A vector (yaw, pitch, roll) of intrinsic Z-Y'-X'' Euler angles represents the following sequence of rotations from child to parent:
 *
 *      rotationChildToParent = R_z(yaw) R_y(pitch) R_x(roll)
 *                            = [ cos(yaw)  -sin(yaw)  0 ] [ cos(pitch)   0  sin(pitch) ] [ 1  0         0          ]
 *                              [ sin(yaw)  cos(yaw)   0 ] [ 0            1  0          ] [ 0  cos(roll) -sin(roll) ]
 *                              [ 0         0          1 ] [ -sin(pitch)  0  cos(pitch) ] [ 0  sin(roll) cos(roll)  ]
 *
 *  where R_k(angle) is the rotation matrix of a positive rotation of @c angle around the k-axis. Starting from the parent frame and
 *  applying active rotations, we first rotate the frame x-y-z by @c yaw around the z-axis to obtain a frame x'-y'-z', then by @c pitch
 *  around the y'-axis to obtain x''-y''-z'', and finally by @c roll around the x''-axis to obtain the child frame x'''-y'''-z'''.
 *
 *  @note This definition by intrinsic Z-Y'-X'' rotations is equivalent to (i.e. yields the same rotation matrix as) the definition by
 *  extrinsic x-y-z rotations where rotations are applied in the reverse order and using the fixed axes of the parent frame. This matches
 *  the convention used in the URDF documentation <https://wiki.ros.org/urdf/XML/joint>: "rotation around fixed axis: first roll around x,
 *  then pitch around y and finally yaw around z".
 *
 *  See also:
 *  - Section "Euler Angles ZYX <=> Rotation Matrix" in https://github.com/ANYbotics/kindr/blob/master/doc/cheatsheet/cheatsheet_latest.pdf
 *  - https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Rotation_matrices
 *  - https://en.wikipedia.org/wiki/Active_and_passive_transformation
 *
 *  The following typedefs are provided for convenience:
 *   - \ref EulerAnglesZyxAD "EulerAnglesZyxD" for active rotation and double primitive type
 *   - \ref EulerAnglesZyxAF "EulerAnglesZyxF" for active rotation and float primitive type
 *   - EulerAnglesYprD = EulerAnglesZyxD
 *   - EulerAnglesYprF = EulerAnglesZyxF
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *  \ingroup rotations
 */
template<typename PrimType_>
class EulerAnglesZyx : public RotationBase<EulerAnglesZyx<PrimType_>> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

  /*! \brief vector of Euler angles [yaw; pitch; roll]
   */
  Base zyx_;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief Euler angles as 3x1-matrix
   */
  typedef Base Vector;

  /*! \brief Default constructor using identity rotation.
   */
  EulerAnglesZyx()
    : zyx_(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param yaw      first rotation angle around Z axis
   *  \param pitch    second rotation angle around Y' axis
   *  \param roll     third rotation angle around X'' axis
   */
  EulerAnglesZyx(Scalar yaw, Scalar pitch, Scalar roll)
    : zyx_(yaw,pitch,roll) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,3,1> [yaw; pitch; roll]
   */
  explicit EulerAnglesZyx(const Base& other)
    : zyx_(other) {
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit EulerAnglesZyx(const RotationBase<OtherDerived_>& other)
    : zyx_(internal::ConversionTraits<EulerAnglesZyx, OtherDerived_>::convert(other.derived()).toImplementation()) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OtherDerived_>
  EulerAnglesZyx& operator =(const RotationBase<OtherDerived_>& other) {
    this->toImplementation() = internal::ConversionTraits<EulerAnglesZyx, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }


  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  EulerAnglesZyx& operator ()(const RotationBase<OtherDerived_>& other) {
    this->toImplementation() = internal::ConversionTraits<EulerAnglesZyx, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  EulerAnglesZyx inverted() const {
    return EulerAnglesZyx(RotationQuaternion<PrimType_>(*this).inverted());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  EulerAnglesZyx& invert() {
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
    return static_cast<Base&>(zyx_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base& toImplementation() const {
    return static_cast<const Base&>(zyx_);
  }

  /*! \brief Gets yaw (Z) angle.
   *  \returns yaw angle (scalar)
   */
  inline Scalar yaw() const {
    return zyx_(0);
  }

  /*! \brief Gets pitch (Y') angle.
   *  \returns pitch angle (scalar)
   */
  inline Scalar pitch() const {
    return zyx_(1);
  }

  /*! \brief Gets roll (X'') angle.
   *  \returns roll angle (scalar)
   */
  inline Scalar roll() const {
    return zyx_(2);
  }

  /*! \brief Sets yaw (Z) angle.
   */
  inline void setYaw(Scalar yaw) {
    zyx_(0) = yaw;
  }

  /*! \brief Sets pitch (Y') angle.
   */
  inline void setPitch(Scalar pitch) {
    zyx_(1) = pitch;
  }

  /*! \brief Sets roll (X'') angle.
   */
  inline void setRoll(Scalar roll) {
    zyx_(2) = roll;
  }

  /*! \brief Reading access to yaw (Z) angle.
   *  \returns yaw angle (scalar) with reading access
   */
  inline Scalar z() const {
    return zyx_(0);
  }

  /*! \brief Reading access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with reading access
   */
  inline Scalar y() const {
    return zyx_(1);
  }

  /*! \brief Reading access to roll (X'') angle.
   *  \returns roll angle (scalar) with reading access
   */
  inline Scalar x() const {
    return zyx_(2);
  }

  /*! \brief Writing access to yaw (Z) angle.
   *  \returns yaw angle (scalar) with writing access
   */
  inline void setZ(Scalar z) {
    zyx_(0) = z;
  }

  /*! \brief Writing access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with writing access
   */
  inline void setY(Scalar y) {
    zyx_(1) = y;
  }

  /*! \brief Writing access to roll (X'') angle.
   *  \returns roll angle (scalar) with writing access
   */
  inline void setX(Scalar x) {
    zyx_(2) = x;
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  EulerAnglesZyx& setIdentity() {
    zyx_.setZero();
    return *this;
  }

  /*! \brief Get identity rotation.
   *  \returns identity rotation
   */
  static EulerAnglesZyx Identity() {
    return EulerAnglesZyx(Base::Zero());
  }

//  /*! \brief Returns a unique Euler angles rotation with angles in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
//   *  This function is used to compare different rotations.
//   *  \returns copy of the Euler angles rotation which is unique
//   */
//  EulerAnglesZyx getUnique() const {
//    //return EulerAnglesZyx<Scalar>(RotationQuaternion<Scalar>(*this).getUnique());
//    Base zyx(kindr::floatingPointModulo(z()+M_PI,2*M_PI)-M_PI,
//             kindr::floatingPointModulo(y()+M_PI,2*M_PI)-M_PI,
//             kindr::floatingPointModulo(x()+M_PI,2*M_PI)-M_PI); // wrap all angles into [-pi,pi)
//
//    const double tol = 1e-3;
//    Scalar& z = zyx(0);
//    Scalar& y = zyx(1);
//    Scalar& x = zyx(2);
//
//    // wrap angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
//    if(y < -M_PI/2 - tol)
//    {
//      if(z < 0) {
//        z = z + M_PI;
//      } else {
//        z = z - M_PI;
//      }
//
//      y = -(y + M_PI);
//
//      if(x < 0) {
//        x = x + M_PI;
//      } else {
//        x = x - M_PI;
//      }
//    }
//    else if(-M_PI/2 - tol <= y && y <= -M_PI/2 + tol)
//    {
//      z += x;
//      x = Scalar(0.0);
//    }
//    else if(-M_PI/2 + tol < y && y < M_PI/2 - tol)
//    {
//      // ok
//    }
//    else if(M_PI/2 - tol <= y && y <= M_PI/2 + tol)
//    {
//      // todo: M_PI/2 should not be in range, other formula?
//      z -= x;
//      x = 0;
//    }
//    else // M_PI/2 + tol < zyx.y()
//    {
//      if(z < 0) {
//        z = z + M_PI;
//      } else {
//        z = z - M_PI;
//      }
//
//      y = -(y - M_PI);
//
//      if(x < 0) {
//        x = x + M_PI;
//      } else {
//        x = x - M_PI;
//      }
//    }
//
//    return EulerAnglesZyx(zyx);
//  }


  /*! \brief Returns a unique Euler angles rotation with angles in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  This function is used to compare different rotations.
   *  \returns copy of the Euler angles rotation which is unique
   */
  EulerAnglesZyx getUnique() const {
    Base zyx(kindr::floatingPointModulo(z()+M_PI,2*M_PI)-M_PI,
             kindr::floatingPointModulo(y()+M_PI,2*M_PI)-M_PI,
             kindr::floatingPointModulo(x()+M_PI,2*M_PI)-M_PI); // wrap all angles into [-pi,pi)

    const double tol = 1e-3;

    // wrap angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    if(zyx.y() < -M_PI/2 - tol)
    {
      if(zyx.x() < 0) {
        zyx.x() = zyx.x() + M_PI;
      } else {
        zyx.x() = zyx.x() - M_PI;
      }

      zyx.y() = -(zyx.y() + M_PI);

      if(zyx.z() < 0) {
        zyx.z() = zyx.z() + M_PI;
      } else {
        zyx.z() = zyx.z() - M_PI;
      }
    }
    else if(-M_PI/2 - tol <= zyx.y() && zyx.y() <= -M_PI/2 + tol)
    {
      zyx.x() -= zyx.z();
      zyx.z() = 0;
    }
    else if(-M_PI/2 + tol < zyx.y() && zyx.y() < M_PI/2 - tol)
    {
      // ok
    }
    else if(M_PI/2 - tol <= zyx.y() && zyx.y() <= M_PI/2 + tol)
    {
      // todo: M_PI/2 should not be in range, other formula?
      zyx.x() += zyx.z();
      zyx.z() = 0;
    }
    else // M_PI/2 + tol < zyx.y()
    {
      if(zyx.x() < 0) {
        zyx.x() = zyx.x() + M_PI;
      } else {
        zyx.x() = zyx.x() - M_PI;
      }

      zyx.y() = -(zyx.y() - M_PI);

      if(zyx.z() < 0) {
        zyx.z() = zyx.z() + M_PI;
      } else {
        zyx.z() = zyx.z() - M_PI;
      }
    }

    return EulerAnglesZyx(zyx);
  }

  /*! \brief Modifies the Euler angles rotation such that the angles lie in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  \returns reference
   */
  EulerAnglesZyx& setUnique() {  // wraps angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    *this = getUnique();
    return *this;
  }


  typename Eigen::Matrix<PrimType_, 3, 3> getMappingFromDiffToLocalAngularVelocity() const {
    using std::sin;
    using std::cos;
    Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
    const PrimType_ x = this->x();
    const PrimType_ y = this->y();
    const PrimType_ z = this->z();
    const PrimType_ t2 = cos(x);
    const PrimType_ t3 = cos(y);
    const PrimType_ t4 = sin(x);
    mat(0, 0) = -sin(y);
    mat(0, 2) = 1.0;
    mat(1, 0) = t3*t4;
    mat(1, 1) = t2;
    mat(2, 0) = t2*t3;
    mat(2, 1) = -t4;
    return mat;
  }

  typename Eigen::Matrix<PrimType_, 3, 3> getMappingFromLocalAngularVelocityToDiff() const {
    using std::sin;
    using std::cos;
    Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
    const PrimType_ x = this->x();
    const PrimType_ y = this->y();
    const PrimType_ z = this->z();
    const PrimType_ t2 = cos(y);
    const PrimType_ t3 = 1.0/t2;
    KINDR_ASSERT_TRUE(std::runtime_error, t2 != PrimType_(0.0), "Gimbal lock: cos(y) is zero!");
    const PrimType_ t4 = cos(x);
    const PrimType_ t5 = sin(x);
    const PrimType_ t6 = sin(y);
    mat(0,1) = t3*t5;
    mat(0,2) = t3*t4;
    mat(1,1) = t4;
    mat(1,2) = -t5;
    mat(2,0) = 1.0;
    mat(2,1) = t3*t5*t6;
    mat(2,2) = t3*t4*t6;
    return mat;
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using RotationBase<EulerAnglesZyx<PrimType_>>::operator*;

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator==.
   *  \returns true if two rotations are equal.
   */
  using RotationBase<EulerAnglesZyx<PrimType_>>::operator==;

  /*! \brief Inequivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator!=.
   *  \returns true if two rotations are not equal.
   */
  using RotationBase<EulerAnglesZyx<PrimType_>>::operator!=;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const EulerAnglesZyx& zyx) {
    out << zyx.toImplementation().transpose();
    return out;
  }
};

//! \brief Active Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with double primitive type
typedef EulerAnglesZyx<double>  EulerAnglesZyxD;
//! \brief Active Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with float primitive type
typedef EulerAnglesZyx<float>  EulerAnglesZyxF;
//! \brief Passive Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with double primitive type
typedef EulerAnglesZyx<double> EulerAnglesZyxPD;
//! \brief Passive Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with float primitive type
typedef EulerAnglesZyx<float> EulerAnglesZyxPF;

//! \brief Equivalent Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) class
template <typename PrimType_>
using EulerAnglesYpr = EulerAnglesZyx<PrimType_>;

//! \brief Active Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with double primitive type
typedef EulerAnglesYpr<double>  EulerAnglesYprD;
//! \brief Active Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with float primitive type
typedef EulerAnglesYpr<float>  EulerAnglesYprF;
//! \brief Passive Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with double primitive type
typedef EulerAnglesYpr<double> EulerAnglesYprPD;
//! \brief Passive Euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with float primitive type
typedef EulerAnglesYpr<float> EulerAnglesYprPF;



namespace internal {

template<typename PrimType_>
class get_scalar<EulerAnglesZyx<PrimType_>> {
 public:
  typedef PrimType_ Scalar;
};

template<typename PrimType_>
class get_matrix3X<EulerAnglesZyx<PrimType_>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesZyx<DestPrimType_>, AngleAxis<SourcePrimType_>> {
 public:
  inline static EulerAnglesZyx<DestPrimType_> convert(const AngleAxis<SourcePrimType_>& aa) {
    return EulerAnglesZyx<DestPrimType_>(RotationQuaternion<DestPrimType_>(aa));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesZyx<DestPrimType_>, RotationMatrix<SourcePrimType_>> {
 public:
  inline static EulerAnglesZyx<DestPrimType_> convert(const RotationMatrix<SourcePrimType_>& R) {
    return EulerAnglesZyx<DestPrimType_>(RotationQuaternion<DestPrimType_>(R));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesZyx<DestPrimType_>, RotationVector<SourcePrimType_>> {
 public:
  inline static EulerAnglesZyx<DestPrimType_> convert(const RotationVector<SourcePrimType_>& rotationVector) {
    return EulerAnglesZyx<DestPrimType_>(RotationQuaternion<SourcePrimType_>(rotationVector));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesZyx<DestPrimType_>, RotationQuaternion<SourcePrimType_>> {
 public:
  inline static EulerAnglesZyx<DestPrimType_> convert(const RotationQuaternion<SourcePrimType_>& q) {
    const Eigen::Matrix<DestPrimType_, 3, 1> vec = (q.toImplementation().toRotationMatrix().eulerAngles(2, 1, 0)).template cast<DestPrimType_>();
    return EulerAnglesZyx<DestPrimType_>(vec(0), vec(1), vec(2));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesZyx<DestPrimType_>, EulerAnglesXyz<SourcePrimType_>> {
 public:
  inline static EulerAnglesZyx<DestPrimType_> convert(const EulerAnglesXyz<SourcePrimType_>& xyz) {
    return EulerAnglesZyx<DestPrimType_>(RotationQuaternion<DestPrimType_>(xyz));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesZyx<DestPrimType_>, EulerAnglesZyx<SourcePrimType_>> {
 public:
  inline static EulerAnglesZyx<DestPrimType_> convert(const EulerAnglesZyx<SourcePrimType_>& zyx) {
    return EulerAnglesZyx<DestPrimType_>(zyx.toImplementation().template cast<DestPrimType_>());
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Multiplication Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Comparison Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


} // namespace internal
} // namespace kindr


