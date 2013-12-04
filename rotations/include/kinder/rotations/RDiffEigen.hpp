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

#ifndef KINDER_ROTATIONS_RDIFFEIGEN_HPP_
#define KINDER_ROTATIONS_RDIFFEIGEN_HPP_


#include <Eigen/Core>

#include "kinder/common/common.hpp"
#include "kinder/common/assert_macros_eigen.hpp"
#include "kinder/rotations/RDiffBase.hpp"
#include "kinder/quaternions/QuaternionEigen.hpp"

namespace kinder {
namespace rotations {
namespace eigen_implementation {

/*! \class AngularVelocity3
 * \brief Angular velocity in 3D-space.
 *
 * This class implements an angular velocity of a body in 3D-space.
 * More precisely an interface to store and access the components of an angular velocity of a rigid body in 3D-space is provided.
 * \tparam PrimType_  Primitive type of the coordinates.
 * \ingroup rotations
 */
template<typename PrimType_>
class AngularVelocity3 : public AngularVelocity3Base<AngularVelocity3<PrimType_>>, private Eigen::Matrix<PrimType_, 3, 1> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;
 public:
  /*! \brief The implementation type.
   *
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;

  /*! \brief The primitive type of the velocities.
   */
  typedef PrimType_ Scalar;

  /*! \brief Default constructor initializes all velocities with zero.
   */
  AngularVelocity3()
    : Base(Base::Zero()) {
  }

  /*! Constructor with three components (x,y,z)
   * \param x   x-coordinate
   * \param y   y-coordinate
   * \param z   z-coordinate
   */
  AngularVelocity3(const PrimType_& x, const PrimType_& y, const PrimType_& z)
    : Base(x, y, z) {
  }


  /*! \brief Constructor using Eigen::Vector3.
   *  \param other   Eigen::Matrix<PrimType_,3,1>
   */
  explicit AngularVelocity3(const Base& other)
    : Base(other) {
   }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return static_cast<const Implementation&>(*this);
  }

  /*!\brief Get x-coordinate of the angular velocity
   * \returns the x-coordinate of the angular velocity
   */
  using Base::x;

  /*!\brief Get y-coordinate of the angular velocity
   * \returns the y-coordinate of the angular velocity
   */
  using Base::y;

  /*!\brief Get z-coordinate of the angular velocity
   * \returns the z-coordinate of the angular velocity
   */
  using Base::z;

  /*! \brief Addition of two angular velocities.
   */
  using AngularVelocity3Base<AngularVelocity3<PrimType_>>::operator+; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Subtraction of two angular velocities.
   */
  using AngularVelocity3Base<AngularVelocity3<PrimType_>>::operator-; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Addition of two angular velocities.
   * \param other   other angular velocity
   */
  template<typename Other_>
  AngularVelocity3<PrimType_>& operator +=(const Other_& other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  /*! \brief Subtraction of two angular velocities.
   * \param other   other angular velocity
   */
  template<typename Other_>
  AngularVelocity3<PrimType_>& operator -=(const Other_& other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  /*! \brief Sets all components of the angular velocity to zero.
   * \returns reference
   */
  AngularVelocity3<PrimType_>& setZero() {
    Base::setZero();
    return *this;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const AngularVelocity3& velocity) {
    out << velocity.toImplementation().transpose();
    return out;
  }
};


//! \brief 3D angular velocity with primitive type double
typedef AngularVelocity3<double>  AngularVelocity3D;

//! \brief 3D angular velocity with primitive type float
typedef AngularVelocity3<float>  AngularVelocity3F;



template<typename PrimType_>
class RotationQuaternionDiff : public RDiffBase<RotationQuaternionDiff<PrimType_>>, public quaternions::eigen_implementation::Quaternion<PrimType_> {
 private:
  /*! \brief The base type.
   */
  typedef quaternions::eigen_implementation::Quaternion<PrimType_> Base;

 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef typename Base::Implementation Implementation;
  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  RotationQuaternionDiff()
    : Base(Implementation::Zero()) {
  }

  /*! \brief Sets all time derivatives to zero.
   *  \returns reference
   */
  RotationQuaternionDiff& setZero() {
    this->Base::setZero();
    return *this;
  }


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationQuaternionDiff& diff) {
    out << diff.toImplementation().transpose();
    return out;
  }
};


//! \brief Time derivative of a rotation quaternion with primitive type double
typedef RotationQuaternionDiff<double> RotationQuaternionDiffD;

//! \brief Time derivative of a rotation quaternion with primitive type float
typedef RotationQuaternionDiff<float> RotationQuaternionDiffF;

/*! \class EulerAnglesZyxDiff
 * \brief Implementation of time derivatives of Euler angles (Z,Y',X'' / yaw,pitch,roll) based on Eigen::Matrix
 *
 * The following two typedefs are provided for convenience:
 *   - EulerAnglesZyxD for primitive type double
 *   - EulerAnglesZyxF for primitive type float
 * \tparam PrimType_ the primitive type of the data (double or float)
 * \ingroup rotations
 */
template<typename PrimType_>
class EulerAnglesZyxDiff : public EulerAnglesDiffBase<EulerAnglesZyxDiff<PrimType_>>,  private Eigen::Matrix<PrimType_, 3, 1>  {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

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
    : Base(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param yaw      time derivative of first rotation angle around Z axis
   *  \param pitch    time derivative of second rotation angle around Y' axis
   *  \param roll     time derivative of third rotation angle around X'' axis
   */
  EulerAnglesZyxDiff(const Scalar& yaw, const Scalar& pitch, const Scalar& roll)
    : Base(yaw,pitch,roll) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,3,1> [roll; pitch; yaw]
   */
  explicit EulerAnglesZyxDiff(const Base& other)
    : Base(other) {
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Base& toImplementation() {
    return static_cast<Base&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base& toImplementation() const {
    return static_cast<const Base&>(*this);
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
     this->Implementation::setZero();
     return *this;
   }

   /*! \brief Addition of two angular velocities.
    */
   using EulerAnglesDiffBase<EulerAnglesZyxDiff<PrimType_>>::operator+; // otherwise ambiguous EulerAnglesDiffBase and Eigen

   /*! \brief Subtraction of two angular velocities.
    */
   using EulerAnglesDiffBase<EulerAnglesZyxDiff<PrimType_>>::operator-; // otherwise ambiguous EulerAnglesDiffBase and Eigen


   /*! \brief Used for printing the object with std::cout.
    *  \returns std::stream object
    */
   friend std::ostream& operator << (std::ostream& out, const EulerAnglesZyxDiff& diff) {
     out << diff.toImplementation().transpose();
     return out;
   }
};

//! \brief Time derivative of Euler angles with z-y-x convention and primitive type double
typedef EulerAnglesZyxDiff<double> EulerAnglesZyxDiffD;

//! \brief Time derivative of Euler angles with z-y-x convention and primitive type float
typedef EulerAnglesZyxDiff<float> EulerAnglesZyxDiffF;


/*! \class EulerAnglesXyzDiff
 * \brief Implementation of time derivatives of Euler angles (X,Y',Z'' / roll,pitch,yaw) based on Eigen::Matrix
 *
 * The following two typedefs are provided for convenience:
 *   - EulerAnglesXyzD for primitive type double
 *   - EulerAnglesXyzF for primitive type float
 * \tparam PrimType_ the primitive type of the data (double or float)
 * \ingroup rotations
 */
template<typename PrimType_>
class EulerAnglesXyzDiff : public EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_>>,  private Eigen::Matrix<PrimType_, 3, 1>  {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

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
    : Base(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param roll     time derivative of first rotation angle around X axis
   *  \param pitch    time derivative of second rotation angle around Y' axis
   *  \param yaw      time derivative of third rotation angle around Z'' axis
   */
  EulerAnglesXyzDiff(const Scalar& yaw, const Scalar& pitch, const Scalar& roll)
    : Base(yaw,pitch,roll) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,3,1> [roll; pitch; yaw]
   */
  explicit EulerAnglesXyzDiff(const Base& other)
    : Base(other) {
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Base& toImplementation() {
    return static_cast<Base&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base& toImplementation() const {
    return static_cast<const Base&>(*this);
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
     this->Implementation::setZero();
     return *this;
   }


   using EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_>>::operator+; // otherwise ambiguous EulerAnglesDiffBase and Eigen


   using EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_>>::operator-; // otherwise ambiguous EulerAnglesDiffBase and Eigen


   /*! \brief Used for printing the object with std::cout.
    *  \returns std::stream object
    */
   friend std::ostream& operator << (std::ostream& out, const EulerAnglesXyzDiff& diff) {
     out << diff.toImplementation().transpose();
     return out;
   }
};

//! \brief Time derivative of Euler angles with x-y-z convention and primitive type double
typedef EulerAnglesXyzDiff<double> EulerAnglesXyzDiffD;

//! \brief Time derivative of Euler angles with x-y-z convention and primitive type float
typedef EulerAnglesXyzDiff<float> EulerAnglesXyzDiffF;



} // namespace eigen_implementation

namespace internal {





} // namespace internal
} // namespace rotations
} // namespace kinder


#endif /* KINDER_ROTATIONS_RDIFFEIGEN_HPP_ */
