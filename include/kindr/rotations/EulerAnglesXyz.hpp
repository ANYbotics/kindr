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


/*! \class EulerAnglesXyz
 *  \brief Implementation of Euler angles (X-Y'-Z'' / roll-pitch-yaw) rotation based on Eigen::Matrix<Scalar,3,1>
 *
 *  The following typedefs are provided for convenience:
 *   - \ref EulerAnglesXyzD "EulerAnglesXyzAD" for double primitive type
 *   - \ref EulerAnglesXyzF "EulerAnglesXyzAF" for float primitive type
 *   - EulerAnglesRpyD = EulerAnglesXyzD
 *   - EulerAnglesRpyF = EulerAnglesXyzF
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *
 *  \ingroup rotations
 */
template<typename PrimType_>
class EulerAnglesXyz : public RotationBase<EulerAnglesXyz<PrimType_>> {
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
  inline explicit EulerAnglesXyz(const RotationBase<OtherDerived_>& other)
    : xyz_(internal::ConversionTraits<EulerAnglesXyz, OtherDerived_>::convert(other.derived()).toImplementation()) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OtherDerived_>
  EulerAnglesXyz& operator =(const RotationBase<OtherDerived_>& other) {
    this->toImplementation() = internal::ConversionTraits<EulerAnglesXyz, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  EulerAnglesXyz& operator ()(const RotationBase<OtherDerived_>& other) {
    this->toImplementation() = internal::ConversionTraits<EulerAnglesXyz, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  EulerAnglesXyz inverted() const {
    return EulerAnglesXyz(RotationQuaternion<PrimType_>(*this).inverted());
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

  /*! \brief Get identity rotation.
   * \returns identity rotation
   */
  static EulerAnglesXyz Identity() {
    return EulerAnglesXyz(Base::Zero());
  }

  /*! \brief Returns a unique Euler angles rotation with angles in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  This function is used to compare different rotations.
   *  \returns copy of the Euler angles rotation which is unique
   */
  EulerAnglesXyz getUnique() const {
    Base xyz(kindr::floatingPointModulo(x()+M_PI,2*M_PI)-M_PI,
             kindr::floatingPointModulo(y()+M_PI,2*M_PI)-M_PI,
             kindr::floatingPointModulo(z()+M_PI,2*M_PI)-M_PI); // wrap all angles into [-pi,pi)

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

  typename Eigen::Matrix<PrimType_, 3, 3> getMappingFromDiffToLocalAngularVelocity() const {
    using std::sin;
    using std::cos;
    Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
    const PrimType_ x = this->x();
    const PrimType_ y = this->y();
    const PrimType_ z = this->z();

    const PrimType_ t2 = cos(y);
    const PrimType_ t3 = sin(z);
    const PrimType_ t4 = cos(z);
    mat(0, 0) = t2*t4;
    mat(0, 1) = t3;
    mat(1, 0) = -t2*t3;
    mat(1, 1) = t4;
    mat(2, 0) = sin(y);
    mat(2, 2) = 1.0;

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
    KINDR_ASSERT_TRUE(std::runtime_error, t2 != PrimType_(0.0), "Gimbal lock: cos(y) is zero!");
    const PrimType_ t3 = 1.0/t2;
    const PrimType_ t4 = sin(z);
    const PrimType_ t5 = cos(z);
    const PrimType_ t6 = sin(y);
    mat(0,0) = t3*t5;
    mat(0,1) = -t3*t4;
    mat(1,0) = t4;
    mat(1,1) = t5;
    mat(2,0) = -t3*t5*t6;
    mat(2,1) = t3*t4*t6;
    mat(2,2) = 1.0;

    return mat;
  }

  typename Eigen::Matrix<PrimType_, 3, 3> getMappingFromGlobalAngularVelocityToDiff() const {
    using std::sin;
    using std::cos;
    Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
    const PrimType_ x = this->x();
    const PrimType_ y = this->y();
    const PrimType_ z = this->z();
    const PrimType_ t2 = cos(y);
    KINDR_ASSERT_TRUE(std::runtime_error, t2 != PrimType_(0.0), "Gimbal lock: cos(y) is zero!");
    const PrimType_ t3 = 1.0/t2;
    const PrimType_ t4 = sin(y);
    const PrimType_ t5 = cos(x);
    const PrimType_ t6 = sin(x);
    mat(0,0) = 1.0;
    mat(0,1) = t3*t4*t6;
    mat(0,2) = -t3*t4*t5;
    mat(1,1) = t5;
    mat(1,2) = t6;
    mat(2,1) = -t3*t6;
    mat(2,2) = t3*t5;
    return mat;
  }


  typename Eigen::Matrix<PrimType_, 3, 3> getMappingFromDiffToGlobalAngularVelocity() const {
    using std::sin;
    using std::cos;
    Eigen::Matrix<PrimType_, 3, 3> mat =  Eigen::Matrix<PrimType_, 3, 3>::Zero();
    const PrimType_ x = this->x();
    const PrimType_ y = this->y();
    const PrimType_ z = this->z();
    const PrimType_ t2 = sin(x);
    const PrimType_ t3 = cos(x);
    const PrimType_ t4 = cos(y);
    mat(0,0) = 1.0;
    mat(0,2) = sin(y);
    mat(1,1) = t3;
    mat(1,2) = -t2*t4;
    mat(2,1) = t2;
    mat(2,2) = t3*t4;
    return mat;
  }


  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using RotationBase<EulerAnglesXyz<PrimType_>>::operator*;

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator==.
   *  \returns true if two rotations are equal.
   */
  using RotationBase<EulerAnglesXyz<PrimType_>>::operator==;

  /*! \brief Inequivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator!=.
   *  \returns true if two rotations are not equal.
   */
  using RotationBase<EulerAnglesXyz<PrimType_>>::operator!=;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const EulerAnglesXyz& xyz) {
    out << xyz.toImplementation().transpose();
    return out;
  }
};

//! \brief Active Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primitive type
typedef EulerAnglesXyz<double>  EulerAnglesXyzD;
//! \brief Active Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primitive type
typedef EulerAnglesXyz<float>  EulerAnglesXyzF;
//! \brief Passive Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primitive type
typedef EulerAnglesXyz<double> EulerAnglesXyzPD;
//! \brief Passive Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primitive type
typedef EulerAnglesXyz<float> EulerAnglesXyzPF;

//! \brief Equivalent Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) class
template <typename PrimType_>
using EulerAnglesRpy = EulerAnglesXyz<PrimType_>;

//! \brief Active Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primitive type
typedef EulerAnglesRpy<double>  EulerAnglesRpyD;
//! \brief Active Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primitive type
typedef EulerAnglesRpy<float>  EulerAnglesRpyF;
//! \brief Passive Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primitive type
typedef EulerAnglesRpy<double> EulerAnglesRpyPD;
//! \brief Passive Euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primitive type
typedef EulerAnglesRpy<float> EulerAnglesRpyPF;



namespace internal {

template<typename PrimType_>
class get_scalar<EulerAnglesXyz<PrimType_>> {
 public:
  typedef PrimType_ Scalar;
};

template<typename PrimType_>
class get_matrix3X<EulerAnglesXyz<PrimType_>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesXyz<DestPrimType_>, AngleAxis<SourcePrimType_>> {
 public:
  inline static EulerAnglesXyz<DestPrimType_> convert(const AngleAxis<SourcePrimType_>& aa) {
//    return EulerAnglesXyz<DestPrimType_>(getRpyFromAngleAxis<SourcePrimType_, DestPrimType_>(aa.toImplementation()));
    return EulerAnglesXyz<DestPrimType_>(RotationQuaternion<DestPrimType_>(aa));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesXyz<DestPrimType_>, RotationVector<SourcePrimType_>> {
 public:
  inline static EulerAnglesXyz<DestPrimType_> convert(const RotationVector<SourcePrimType_>& rotationVector) {
    return EulerAnglesXyz<DestPrimType_>(AngleAxis<SourcePrimType_>(rotationVector));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesXyz<DestPrimType_>, RotationQuaternion<SourcePrimType_>> {
 public:
  inline static EulerAnglesXyz<DestPrimType_> convert(const RotationQuaternion<SourcePrimType_>& q) {
     Eigen::Matrix<DestPrimType_, 3, 1>  vec = q.toImplementation().toRotationMatrix().eulerAngles(0, 1, 2).template cast<DestPrimType_>();
     return EulerAnglesXyz<DestPrimType_>(vec(0), vec(1), vec(2));
  }
};


template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesXyz<DestPrimType_>, RotationMatrix<SourcePrimType_>> {
 public:
  inline static EulerAnglesXyz<DestPrimType_> convert(const RotationMatrix<SourcePrimType_>& R) {
    return EulerAnglesXyz<DestPrimType_>(RotationQuaternion<DestPrimType_>(R));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesXyz<DestPrimType_>, EulerAnglesXyz<SourcePrimType_>> {
 public:
  inline static EulerAnglesXyz<DestPrimType_> convert(const EulerAnglesXyz<SourcePrimType_>& xyz) {
    return EulerAnglesXyz<DestPrimType_>(xyz.toImplementation().template cast<DestPrimType_>());
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<EulerAnglesXyz<DestPrimType_>, EulerAnglesZyx<SourcePrimType_>> {
 public:
  inline static EulerAnglesXyz<DestPrimType_> convert(const EulerAnglesZyx<SourcePrimType_>& zyx) {
    return EulerAnglesXyz<DestPrimType_>(RotationQuaternion<DestPrimType_>(zyx));
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
