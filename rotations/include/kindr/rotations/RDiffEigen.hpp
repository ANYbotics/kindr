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

#ifndef KINDR_ROTATIONS_RDIFFEIGEN_HPP_
#define KINDR_ROTATIONS_RDIFFEIGEN_HPP_


#include <Eigen/Core>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RDiffBase.hpp"
#include "kindr/rotations/RotationEigen.hpp"
#include "kindr/quaternions/QuaternionEigen.hpp"
#include "kindr/linear_algebra/LinearAlgebra.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {

/*! \class AngularVelocity
 * \brief Angular velocity in 3D-space.
 *
 * This class implements an angular velocity of a body in 3D-space.
 * More precisely an interface to store and access the components of an angular velocity of a rigid body in 3D-space is provided.
 * \tparam PrimType_  Primitive type of the coordinates.
 * \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class AngularVelocity : public AngularVelocityBase<AngularVelocity<PrimType_, Usage_>, Usage_>, private Eigen::Matrix<PrimType_, 3, 1> {
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
  AngularVelocity()
    : Base(Base::Zero()) {
  }

  /*! Constructor with three components (x,y,z)
   * \param x   x-coordinate
   * \param y   y-coordinate
   * \param z   z-coordinate
   */
  AngularVelocity(const PrimType_& x, const PrimType_& y, const PrimType_& z)
    : Base(x, y, z) {
  }


  /*! \brief Constructor using Eigen::Vector3.
   *  \param other   Eigen::Matrix<PrimType_,3,1>
   */
  explicit AngularVelocity(const Base& other)
    : Base(other) {
   }

//  template<typename RotationDerived_, typename OtherDerived_>
//  inline explicit AngularVelocity(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
//    : Base(internal::RDiffConversionTraits<AngularVelocity, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived())){
//  }


  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit AngularVelocity(const RotationDerived_& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : Base(internal::RDiffConversionTraits<AngularVelocity, OtherDerived_, RotationDerived_>::convert(rotation, other.derived())){
  }


//  template<typename RotationDerived_, typename internal::get_other_usage<RotationDerived_>::OtherUsage Other_, typename OtherDerived_>
//  inline explicit AngularVelocity(const typename internal::get_other_usage<RotationDerived_>::OtherUsage& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
////    : Base(internal::RDiffConversionTraits<AngularVelocity, OtherDerived_, RotationDerived_>::convert(rotation, other.derived())){
//  : Base(){
//  }


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
  using AngularVelocityBase<AngularVelocity<PrimType_, Usage_>,Usage_>::operator+; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Subtraction of two angular velocities.
   */
  using AngularVelocityBase<AngularVelocity<PrimType_, Usage_>, Usage_>::operator-; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Addition of two angular velocities.
   * \param other   other angular velocity
   */
  template<typename Other_>
  AngularVelocity& operator +=(const Other_& other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  /*! \brief Subtraction of two angular velocities.
   * \param other   other angular velocity
   */
  template<typename Other_>
  AngularVelocity& operator -=(const Other_& other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  /*! \brief Sets all components of the angular velocity to zero.
   * \returns reference
   */
  AngularVelocity& setZero() {
    Base::setZero();
    return *this;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const AngularVelocity& velocity) {
    out << velocity.toImplementation().transpose();
    return out;
  }
};

//! \brief 3D angular velocity with primitive type double
typedef AngularVelocity<double, RotationUsage::PASSIVE>  AngularVelocityPD;
//! \brief 3D angular velocity with primitive type float
typedef AngularVelocity<float, RotationUsage::PASSIVE>  AngularVelocityPF;
//! \brief 3D angular velocity with primitive type double
typedef AngularVelocity<double, RotationUsage::ACTIVE>  AngularVelocityAD;
//! \brief 3D angular velocity with primitive type float
typedef AngularVelocity<float, RotationUsage::ACTIVE>  AngularVelocityAF;




template<typename PrimType_, enum RotationUsage Usage_>
class RotationQuaternionDiff : public RotationQuaternionDiffBase<RotationQuaternionDiff<PrimType_, Usage_>,Usage_>, private quaternions::eigen_impl::Quaternion<PrimType_> {
 private:
  /*! \brief The base type.
   */
  typedef quaternions::eigen_impl::Quaternion<PrimType_> Base;

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
    : Base() {
  }

  explicit RotationQuaternionDiff(const Base& other) // explicit on purpose
    : Base(other) {
  }

  /*! \brief Constructor using four scalars.
   *  \param w     first entry of the derivative of the quaternion
   *  \param x     second entry of the derivative of the quaternion
   *  \param y     third entry of the derivative of the quaternion
   *  \param z     fourth entry of the derivative of the quaternion
   */
  RotationQuaternionDiff(const Scalar& w, const Scalar& x, const Scalar& y, const Scalar& z)
    : Base(w,x,y,z) {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit RotationQuaternionDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : Base(internal::RDiffConversionTraits<RotationQuaternionDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived())){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RDiffConversionTraits<OtherDerived_, RotationQuaternionDiff, RotationDerived_>::convert(rotation.derived(), *this);
  }


  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return this->toQuaternion().toImplementation();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return this->toQuaternion().toImplementation();
  }

  /*! \brief Cast to the base type.
   *  \returns the quaternion that contains the derivative (recommended only for advanced users)
   */
  Base& toQuaternion()  {
    return static_cast<Base&>(*this);
  }

  /*! \brief Cast to the base type.
   *  \returns the quaternion that contains the derivative (recommended only for advanced users)
   */
  const Base& toQuaternion() const {
    return static_cast<const Base&>(*this);
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
    out << diff.toQuaternion();
    return out;
  }
};


//! \brief Time derivative of a rotation quaternion with primitive type double
typedef RotationQuaternionDiff<double, RotationUsage::PASSIVE> RotationQuaternionDiffPD;
//! \brief Time derivative of a rotation quaternion with primitive type float
typedef RotationQuaternionDiff<float, RotationUsage::PASSIVE> RotationQuaternionDiffPF;
//! \brief Time derivative of a rotation quaternion with primitive type double
typedef RotationQuaternionDiff<double, RotationUsage::ACTIVE> RotationQuaternionDiffAD;
//! \brief Time derivative of a rotation quaternion with primitive type float
typedef RotationQuaternionDiff<float, RotationUsage::ACTIVE> RotationQuaternionDiffAF;




template<typename PrimType_, enum RotationUsage Usage_>
class RotationMatrixDiff : public RotationMatrixDiffBase<RotationMatrixDiff<PrimType_, Usage_>,Usage_>, private Eigen::Matrix<PrimType_, 3, 3> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 3> Base;

 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  RotationMatrixDiff()
    : Base() {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,3,3>
   */
  explicit RotationMatrixDiff(const Base& other) // explicit on purpose
    : Base(other) {
  }

  /*! \brief Constructor using nine scalars.
   *  \param r11     entry in row 1, col 1
   *  \param r12     entry in row 1, col 2
   *  \param r13     entry in row 1, col 3
   *  \param r21     entry in row 2, col 1
   *  \param r22     entry in row 2, col 2
   *  \param r23     entry in row 2, col 3
   *  \param r31     entry in row 3, col 1
   *  \param r32     entry in row 3, col 2
   *  \param r33     entry in row 3, col 3
   */
  RotationMatrixDiff(const Scalar& r11, const Scalar& r12, const Scalar& r13,
                     const Scalar& r21, const Scalar& r22, const Scalar& r23,
                     const Scalar& r31, const Scalar& r32, const Scalar& r33) {
    *this << r11,r12,r13,r21,r22,r23,r31,r32,r33;
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit RotationMatrixDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : Base(internal::RDiffConversionTraits<RotationMatrixDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived())){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RDiffConversionTraits<OtherDerived_, RotationMatrixDiff, RotationDerived_>::convert(rotation.derived(), *this);
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

  /*! \brief Reading access to the rotation matrix.
   *  \returns rotation matrix (matrix) with reading access
   */
  inline const Implementation& matrix() const {
    return this->toImplementation();
  }

  /*! \brief Writing access to the rotation matrix.
   *  \returns rotation matrix (matrix) with writing access
   */
  inline Implementation& matrix() {
    return this->toImplementation();
  }

  /*! \brief Sets all time derivatives to zero.
   *  \returns reference
   */
  RotationMatrixDiff& setZero() {
    this->toImplementation().setZero();
    return *this;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationMatrixDiff& diff) {
    out << diff.toImplementation();
    return out;
  }
};


//! \brief Time derivative of a rotation quaternion with primitive type double
typedef RotationMatrixDiff<double, RotationUsage::PASSIVE> RotationMatrixDiffPD;
//! \brief Time derivative of a rotation quaternion with primitive type float
typedef RotationMatrixDiff<float, RotationUsage::PASSIVE> RotationMatrixDiffPF;
//! \brief Time derivative of a rotation quaternion with primitive type double
typedef RotationMatrixDiff<double, RotationUsage::ACTIVE> RotationMatrixDiffAD;
//! \brief Time derivative of a rotation quaternion with primitive type float
typedef RotationMatrixDiff<float, RotationUsage::ACTIVE> RotationMatrixDiffAF;



/*! \class EulerAnglesZyxDiff
 * \brief Implementation of time derivatives of Euler angles (Z,Y',X'' / yaw,pitch,roll) based on Eigen::Matrix
 *
 * The following two typedefs are provided for convenience:
 *   - EulerAnglesZyxD for primitive type double
 *   - EulerAnglesZyxF for primitive type float
 * \tparam PrimType_ the primitive type of the data (double or float)
 * \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class EulerAnglesZyxDiff : public EulerAnglesDiffBase<EulerAnglesZyxDiff<PrimType_, Usage_>, Usage_>,  private Eigen::Matrix<PrimType_, 3, 1>  {
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

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit EulerAnglesZyxDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : Base(internal::RDiffConversionTraits<EulerAnglesZyxDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived())){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RDiffConversionTraits<OtherDerived_, EulerAnglesZyxDiff, RotationDerived_>::convert(rotation.derived(), *this);
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
     this->toImplementation().setZero();
     return *this;
   }

   /*! \brief Addition of two angular velocities.
    */
   using EulerAnglesDiffBase<EulerAnglesZyxDiff<PrimType_,Usage_>,Usage_>::operator+; // otherwise ambiguous EulerAnglesDiffBase and Eigen

   /*! \brief Subtraction of two angular velocities.
    */
   using EulerAnglesDiffBase<EulerAnglesZyxDiff<PrimType_,Usage_>,Usage_>::operator-; // otherwise ambiguous EulerAnglesDiffBase and Eigen


   /*! \brief Used for printing the object with std::cout.
    *  \returns std::stream object
    */
   friend std::ostream& operator << (std::ostream& out, const EulerAnglesZyxDiff& diff) {
     out << diff.toImplementation().transpose();
     return out;
   }
};

//! \brief Time derivative of Euler angles with z-y-x convention and primitive type double
typedef EulerAnglesZyxDiff<double, RotationUsage::PASSIVE> EulerAnglesZyxDiffPD;
//! \brief Time derivative of Euler angles with z-y-x convention and primitive type float
typedef EulerAnglesZyxDiff<float, RotationUsage::PASSIVE> EulerAnglesZyxDiffPF;
//! \brief Time derivative of Euler angles with z-y-x convention and primitive type double
typedef EulerAnglesZyxDiff<double, RotationUsage::ACTIVE> EulerAnglesZyxDiffAD;
//! \brief Time derivative of Euler angles with z-y-x convention and primitive type float
typedef EulerAnglesZyxDiff<float, RotationUsage::ACTIVE> EulerAnglesZyxDiffAF;

/*! \class EulerAnglesXyzDiff
 * \brief Implementation of time derivatives of Euler angles (X,Y',Z'' / roll,pitch,yaw) based on Eigen::Matrix
 *
 * The following two typedefs are provided for convenience:
 *   - EulerAnglesXyzD for primitive type double
 *   - EulerAnglesXyzF for primitive type float
 * \tparam PrimType_ the primitive type of the data (double or float)
 * \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_ >
class EulerAnglesXyzDiff : public EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_,Usage_>,Usage_>,  private Eigen::Matrix<PrimType_, 3, 1>  {
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


  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit EulerAnglesXyzDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : Base(internal::RDiffConversionTraits<EulerAnglesXyzDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived())){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RDiffConversionTraits<OtherDerived_, EulerAnglesXyzDiff, RotationDerived_>::convert(rotation.derived(), *this);
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
     this->toImplementation().setZero();
     return *this;
   }


   using EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_,Usage_>,Usage_>::operator+; // otherwise ambiguous EulerAnglesDiffBase and Eigen


   using EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_,Usage_>,Usage_>::operator-; // otherwise ambiguous EulerAnglesDiffBase and Eigen


   /*! \brief Used for printing the object with std::cout.
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


template<typename PrimType_, enum RotationUsage Usage_>
class RDiffConversionTraits<eigen_impl::AngularVelocity<PrimType_, Usage_>, eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>, eigen_impl::RotationQuaternion<PrimType_, Usage_>> {
 public:
  inline static eigen_impl::AngularVelocity<PrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<PrimType_, Usage_>& rquat, const eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>& rquatdiff) {
    Eigen::Matrix<PrimType_,3,4> H_bar;
    H_bar << -rquat.toUnitQuaternion().x(),  rquat.toUnitQuaternion().w(),  rquat.toUnitQuaternion().z(), -rquat.toUnitQuaternion().y(),
             -rquat.toUnitQuaternion().y(), -rquat.toUnitQuaternion().z(),  rquat.toUnitQuaternion().w(),  rquat.toUnitQuaternion().x(),
             -rquat.toUnitQuaternion().z(),  rquat.toUnitQuaternion().y(), -rquat.toUnitQuaternion().x(),  rquat.toUnitQuaternion().w();
    return eigen_impl::AngularVelocity<PrimType_, Usage_>(2.0*H_bar*rquatdiff.toQuaternion().getVector4());
  }
};

//template<typename PrimType_, enum RotationUsage Usage_>
//class RDiffConversionTraits<eigen_impl::AngularVelocity<PrimType_, Usage_>, eigen_impl::RotationMatrixDiff<PrimType_, Usage_>, eigen_impl::RotationMatrix<PrimType_, Usage_>> {
// public:
//  inline static eigen_impl::AngularVelocity<PrimType_, Usage_> convert(const eigen_impl::RotationMatrix<PrimType_, Usage_>& rotationMatrix, const eigen_impl::RotationMatrixDiff<PrimType_, Usage_>& rotationMatrixDiff) {
//    return eigen_impl::AngularVelocity<PrimType_, Usage_>(linear_algebra::getVectorFromSkewMatrix<PrimType_>(rotationMatrixDiff.toImplementation()*rotationMatrix.inverted().toImplementation()));
//  }
//};


//! B_\hat{w}_IB = (R_IB*dR_IB')
template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>& rotationMatrix, const eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::ACTIVE>& rotationMatrixDiff) {
    return eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE>(linear_algebra::getVectorFromSkewMatrix<PrimType_>(rotationMatrix.toImplementation()*rotationMatrixDiff.toImplementation().transpose()));
  }
};

//! test
template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>> {
 public:
  inline static eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>& rotationMatrix, const eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::ACTIVE>& rotationMatrixDiff) {
    return eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE>(linear_algebra::getVectorFromSkewMatrix<PrimType_>(rotationMatrix.inverted().toImplementation()*rotationMatrixDiff.toImplementation()));
  }
};

//! B_\hat{w}_IB = (C_IB'*dC_IB)
template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::PASSIVE>, eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>> {
 public:
  inline static eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>& rotationMatrix, const eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::PASSIVE>& rotationMatrixDiff) {
    return eigen_impl::AngularVelocity<PrimType_, RotationUsage::ACTIVE>(linear_algebra::getVectorFromSkewMatrix<PrimType_>(rotationMatrix.inverted().toImplementation()*rotationMatrixDiff.toImplementation()));
  }
};

template<typename PrimType_, enum RotationUsage Usage_>
class RDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>, eigen_impl::AngularVelocity<PrimType_, Usage_>, eigen_impl::RotationQuaternion<PrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationQuaternionDiff<PrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<PrimType_, Usage_>& rquat, const eigen_impl::AngularVelocity<PrimType_, Usage_>& angularVelocity) {
    Eigen::Matrix<PrimType_,4,3> H_bar_transpose;
    H_bar_transpose << -rquat.toUnitQuaternion().x(), -rquat.toUnitQuaternion().y(), -rquat.toUnitQuaternion().z(),
                        rquat.toUnitQuaternion().w(), -rquat.toUnitQuaternion().z(),  rquat.toUnitQuaternion().y(),
                        rquat.toUnitQuaternion().z(),  rquat.toUnitQuaternion().w(), -rquat.toUnitQuaternion().x(),
                       -rquat.toUnitQuaternion().y(),  rquat.toUnitQuaternion().x(),  rquat.toUnitQuaternion().w();
    return eigen_impl::RotationQuaternionDiff<PrimType_, Usage_>(quaternions::eigen_impl::Quaternion<PrimType_>(0.5*H_bar_transpose*angularVelocity.toImplementation()));
  }
};


template<typename PrimType_, enum RotationUsage Usage_>
class RDiffConversionTraits<eigen_impl::EulerAnglesXyzDiff<PrimType_, Usage_>, eigen_impl::AngularVelocity<PrimType_, Usage_>, eigen_impl::EulerAnglesXyz<PrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesXyzDiff<PrimType_, Usage_> convert(const eigen_impl::EulerAnglesXyz<PrimType_, Usage_>& eulerAngles, const eigen_impl::AngularVelocity<PrimType_, Usage_>& angularVelocity) {
    typedef typename Eigen::Matrix<PrimType_, 3, 3> Matrix3x3;

    const PrimType_ alpha = eulerAngles.roll();
    const PrimType_ beta = eulerAngles.pitch();
    const PrimType_ gamma = eulerAngles.yaw();

    Matrix3x3 H = Matrix3x3::Zero();

    if(cos(beta) == 0) {
      H << std::numeric_limits<PrimType_>::max(), std::numeric_limits<PrimType_>::max(), 0, sin(gamma), cos(gamma), 0, -cos(gamma)*tan(beta), sin(gamma)*tan(beta), 1;
    }
    else {
      H << cos(gamma)/cos(beta), -sin(gamma)/cos(beta), 0, sin(gamma), cos(gamma), 0, -cos(gamma)*tan(beta), sin(gamma)*tan(beta), 1;
    }

    return eigen_impl::EulerAnglesXyzDiff<PrimType_, Usage_>(H*angularVelocity.toImplementation());
  }
};



} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_RDIFFEIGEN_HPP_ */
