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
#include "kindr/common/assert_macros.hpp"
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
 * This class implements an angular velocity of a rigid body in 3D-space.
 *
 * Note that only the version with the active usage type makes sense to represent a physical angular velocity of a body.
 * The angular velocity should represent the absolute rotational velocity of a rigid body with respect to an inertial frame I
 * and its coordinates should be expressed in the body fixed frame (\f$\Omega_B = B_\omega_{IB}$\f)
 *
 * \tparam PrimType_  Primitive type of the coordinates.
 * \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class LocalAngularVelocity : public AngularVelocityBase<LocalAngularVelocity<PrimType_, Usage_>, Usage_>, private Eigen::Matrix<PrimType_, 3, 1> {
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
  LocalAngularVelocity()
    : Base(Base::Zero()) {
  }

  /*! Constructor with three components (x,y,z)
   * \param x   x-coordinate
   * \param y   y-coordinate
   * \param z   z-coordinate
   */
  LocalAngularVelocity(Scalar x, Scalar y, Scalar z)
    : Base(x, y, z) {
  }


  /*! \brief Constructor using Eigen::Matrix<Scalar,3,1>.
   *  \param other   Eigen::Matrix<Scalar,3,1>
   */
  explicit LocalAngularVelocity(const Base& other)
    : Base(other) {
   }


  /*! Constructor with a time derivative of a rotation with a different parameterization
   *
   * \param rotation    rotation the time derivative is taken at
   * \param other       other time derivative
   */
  template<typename RotationDerived_, enum RotationUsage RotationUsage_, typename OtherDerived_, enum RotationUsage OtherUsage_>
  inline explicit LocalAngularVelocity(const RotationBase<RotationDerived_, RotationUsage_>& rotation, const RDiffBase<OtherDerived_, OtherUsage_>& other)
    : Base(internal::RDiffConversionTraits<LocalAngularVelocity, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived())){
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
  using AngularVelocityBase<LocalAngularVelocity<PrimType_, Usage_>,Usage_>::operator+; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Subtraction of two angular velocities.
   */
  using AngularVelocityBase<LocalAngularVelocity<PrimType_, Usage_>, Usage_>::operator-; // otherwise ambiguous PositionBase and Eigen

  /*! \brief Addition of two angular velocities.
   * \param other   other angular velocity
   */
  template<typename Other_>
  LocalAngularVelocity& operator +=(const Other_& other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  /*! \brief Subtraction of two angular velocities.
   * \param other   other angular velocity
   */
  template<typename Other_>
  LocalAngularVelocity& operator -=(const Other_& other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  /*! \brief Sets all components of the angular velocity to zero.
   * \returns reference
   */
  LocalAngularVelocity& setZero() {
    Base::setZero();
    return *this;
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const LocalAngularVelocity& velocity) {
    out << velocity.toImplementation().transpose();
    return out;
  }
};

//! \brief 3D angular velocity with primitive type double
//typedef AngularVelocity<double, RotationUsage::PASSIVE>  AngularVelocityPD;
//! \brief 3D angular velocity with primitive type float
//typedef AngularVelocity<float, RotationUsage::PASSIVE>  AngularVelocityPF;
//! \brief 3D angular velocity with primitive type double
typedef LocalAngularVelocity<double, RotationUsage::ACTIVE>  AngularVelocityAD;
//! \brief 3D angular velocity with primitive type float
typedef LocalAngularVelocity<float, RotationUsage::ACTIVE>  AngularVelocityAF;



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

  AngleAxisDiff()
    : Base() {
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


template<typename PrimType_, enum RotationUsage Usage_>
class RotationVectorDiff : public RotationVectorDiffBase<RotationVectorDiff<PrimType_, Usage_>,Usage_> {
 private:
  /*! \brief The base type.
   */
  typedef typename Eigen::Matrix<PrimType_, 3, 1> Base;

 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;

  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief data container
   */
  Base vector_;

  RotationVectorDiff()
    : vector_(Base::Zero()) {
  }

  explicit RotationVectorDiff(const Base& other) // explicit on purpose
    : vector_(other) {
  }

  /*! \brief Constructor using four scalars.
   *  \param v1      first entry of the time derivative of the rotation axis vector
   *  \param v2      second entry of the time derivative of the rotation axis vector
   *  \param v3      third entry of the time derivative of the rotation axis vector
   */
  RotationVectorDiff(Scalar v1, Scalar v2, Scalar v3)
    : vector_(v1,v2,v3) {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit RotationVectorDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : vector_(internal::RDiffConversionTraits<RotationVectorDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toImplementation()){
  }

  /*! \brief Cast to another representation of the time derivative of a rotation
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_, typename RotationDerived_>
  OtherDerived_ cast(const RotationBase<RotationDerived_, Usage_>& rotation) const {
    return internal::RDiffConversionTraits<OtherDerived_, RotationVectorDiff, RotationDerived_>::convert(rotation.derived(), *this);
  }


  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(vector_);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return vector_;
  }


  inline Scalar x() const {
    return toImplementation()(0);
  }


  inline Scalar y() const {
    return toImplementation()(1);
  }


  inline Scalar z() const {
    return toImplementation()(2);
  }


  /*! \brief Sets all time derivatives to zero.
   *  \returns reference
   */
  RotationVectorDiff& setZero() {
    vector_.setZero();
    return *this;
  }


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationVectorDiff& diff) {
    out << diff.toImplementation().transpose();
    return out;
  }
};


//! \brief Time derivative of a rotation vector with primitive type double
typedef RotationVectorDiff<double, RotationUsage::PASSIVE> RotationVectorDiffPD;
//! \brief Time derivative of a rotation vector with primitive type float
typedef RotationVectorDiff<float, RotationUsage::PASSIVE> RotationVectorDiffPF;
//! \brief Time derivative of a rotation vector with primitive type double
typedef RotationVectorDiff<double, RotationUsage::ACTIVE> RotationVectorDiffAD;
//! \brief Time derivative of a rotation vector with primitive type float
typedef RotationVectorDiff<float, RotationUsage::ACTIVE> RotationVectorDiffAF;


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
  RotationQuaternionDiff(Scalar w, Scalar x, Scalar y, Scalar z)
    : Base(w,x,y,z) {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit RotationQuaternionDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : Base(internal::RDiffConversionTraits<RotationQuaternionDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toQuaternion()){
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
  RotationMatrixDiff(Scalar r11, Scalar r12, Scalar r13,
                     Scalar r21, Scalar r22, Scalar r23,
                     Scalar r31, Scalar r32, Scalar r33) {
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
class EulerAnglesZyxDiff : public EulerAnglesDiffBase<EulerAnglesZyxDiff<PrimType_, Usage_>, Usage_> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

  /*! \brief data container [yaw; pitch, roll]
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

  /*! \brief Default constructor.
   */
  EulerAnglesZyxDiff()
    : zyx_(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param yaw      time derivative of first rotation angle around Z axis
   *  \param pitch    time derivative of second rotation angle around Y' axis
   *  \param roll     time derivative of third rotation angle around X'' axis
   */
  EulerAnglesZyxDiff(Scalar yaw, Scalar pitch, Scalar roll)
    : zyx_(yaw,pitch,roll) {
  }

  /*! \brief Constructor using a time derivative with a different parameterization
   *
   * \param rotation  rotation
   * \param other     other time derivative
   */
  template<typename RotationDerived_, typename OtherDerived_>
  inline explicit EulerAnglesZyxDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : zyx_(internal::RDiffConversionTraits<EulerAnglesZyxDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toImplementation()){
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
    : zyx_(other) {
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Base& toImplementation() {
    return zyx_;
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base& toImplementation() const {
    return static_cast<const Base&>(zyx_);
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
class EulerAnglesXyzDiff : public EulerAnglesDiffBase<EulerAnglesXyzDiff<PrimType_,Usage_>,Usage_> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 1> Base;

  /*! \brief data container [roll; pitch, yaw]
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
  inline explicit EulerAnglesXyzDiff(const RotationBase<RotationDerived_, Usage_>& rotation, const RDiffBase<OtherDerived_, Usage_>& other)
    : xyzDiff_(internal::RDiffConversionTraits<EulerAnglesXyzDiff, OtherDerived_, RotationDerived_>::convert(rotation.derived(), other.derived()).toImplementation()){
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


/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>& rquat, const eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>& rquatdiff) {
    Eigen::Matrix<PrimType_,3,4> H_bar;
    H_bar << -rquat.toUnitQuaternion().x(),  rquat.toUnitQuaternion().w(),  rquat.toUnitQuaternion().z(), -rquat.toUnitQuaternion().y(),
             -rquat.toUnitQuaternion().y(), -rquat.toUnitQuaternion().z(),  rquat.toUnitQuaternion().w(),  rquat.toUnitQuaternion().x(),
             -rquat.toUnitQuaternion().z(),  rquat.toUnitQuaternion().y(), -rquat.toUnitQuaternion().x(),  rquat.toUnitQuaternion().w();
    return eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>(2.0*H_bar*rquatdiff.toQuaternion().vector());
  }
};

//! B_\hat{w}_IB = (R_IB*dR_IB')
template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>& rotationMatrix, const eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::ACTIVE>& rotationMatrixDiff) {
    return eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>(linear_algebra::getVectorFromSkewMatrix<PrimType_>(rotationMatrix.toImplementation()*rotationMatrixDiff.toImplementation().transpose()));
  }
};


//! B_\hat{w}_IB = (C_IB'*dC_IB)
template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::PASSIVE>, eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>> {
 public:
  inline static eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>& rotationMatrix, const eigen_impl::RotationMatrixDiff<PrimType_, RotationUsage::PASSIVE>& rotationMatrixDiff) {
    return eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>(linear_algebra::getVectorFromSkewMatrix<PrimType_>(rotationMatrix.inverted().toImplementation()*rotationMatrixDiff.toImplementation()));
  }
};

//! B_\hat{w}_IB = B_n \dot{\theta} + \dot{B_n}*sin(\theta) + B_\hat{n}(1-cos(\theta))
template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::AngleAxisDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::AngleAxis<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::AngleAxis<PrimType_, RotationUsage::ACTIVE>& angleAxis, const eigen_impl::AngleAxisDiff<PrimType_, RotationUsage::ACTIVE>& angleAxisDiff) {
    return eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>(angleAxis.axis()*angleAxisDiff.angle() + angleAxisDiff.axis()*sin(angleAxis.angle()) + linear_algebra::getSkewMatrixFromVector(angleAxis.axis())*angleAxisDiff.axis()*(1-cos(angleAxis.angle())));
  }
};

template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>& rotationVector, const eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>& rotationVectorDiff) {
    typedef typename eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>::Implementation Vector;
    typedef PrimType_ Scalar;
    typedef typename Eigen::Matrix<PrimType_, 3, 3> Matrix3x3;


    // not tested:
//    const Vector rv = rotationVector.toImplementation();
//    const Vector rvDiff = rotationVectorDiff.toImplementation();
//    const Matrix3x3 rv_hat = linear_algebra::getSkewMatrixFromVector(rv);
//    const Scalar angle = rv.norm();
//    const Vector angularVelocity = rvDiff-rv_hat*rvDiff*(1-cos(angle)/(angle*angle)) + rv_hat*rv_hat*rvDiff*((angle-sin(angle))/(angle*angle*angle));
//    return eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>(angularVelocity);

    const PrimType_ v = rotationVector.vector().norm();
    const PrimType_ v1 = rotationVector.firstEntry();
    const PrimType_ v2 = rotationVector.secondEntry();
    const PrimType_ v3 = rotationVector.thirdEntry();
    const PrimType_ dv1 = rotationVectorDiff.x();
    const PrimType_ dv2 = rotationVectorDiff.y();
    const PrimType_ dv3 = rotationVectorDiff.z();

    const PrimType_ t2 = 1.0/(v*v*v);
    const PrimType_ t3 = cos(v);
    const PrimType_ t4 = sin(v);
    const PrimType_ t5 = v1*v1;
    const PrimType_ t6 = v1*v2;
    const PrimType_ t7 = v*v;
    const PrimType_ t8 = t4*t7;
    const PrimType_ t9 = v2*v2;
    const PrimType_ t10 = v2*v3;
    const PrimType_ t11 = v1*v3;
    const PrimType_ t12 = t3*v2;
    const PrimType_ t13 = v3*v3;
    const PrimType_ w1 = dv3*t2*(v*(t11+t12-v2)-t4*v1*v3)+dv1*t2*(t8-t4*t5+t5*v)+dv2*t2*(v*(t6+v3-t3*v3)-t4*v1*v2);
    const PrimType_ w2 = dv1*t2*(v*(t6-v3+t3*v3)-t4*v1*v2)+dv2*t2*(t8-t4*t9+t9*v)+dv3*t2*(v*(t10+v1-t3*v1)-t4*v2*v3);
    const PrimType_ w3 = dv2*t2*(v*(t10-v1+t3*v1)-t4*v2*v3)+dv1*t2*(v*(t11-t12+v2)-t4*v1*v3)+dv3*t2*(t8-t4*t13+t13*v);

    return eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>(w1, w2, w3);

  }
};


template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::EulerAnglesZyxDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::EulerAnglesZyx<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::EulerAnglesZyx<PrimType_, RotationUsage::ACTIVE>& eulerAngles, const eigen_impl::EulerAnglesZyxDiff<PrimType_, RotationUsage::ACTIVE>& eulerAnglesDiff) {
    const PrimType_ phi = eulerAngles.roll();
    const PrimType_ theta = eulerAngles.pitch();
    const PrimType_ dphi = eulerAnglesDiff.roll();
    const PrimType_ dtheta = eulerAnglesDiff.pitch();
    const PrimType_ dpsi = eulerAnglesDiff.yaw();
    const PrimType_ t2 = sin(phi);
    const PrimType_ t3 = cos(phi);
    const PrimType_ t4 = cos(theta);
    return eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>(dphi-dpsi*sin(theta), dtheta*t3+dpsi*t2*t4, -dtheta*t2+dpsi*t3*t4);
  }
};

template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::EulerAnglesXyzDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::EulerAnglesXyz<PrimType_, RotationUsage::ACTIVE>& eulerAngles, const eigen_impl::EulerAnglesXyzDiff<PrimType_, RotationUsage::ACTIVE>& eulerAnglesDiff) {
    const PrimType_ beta = eulerAngles.pitch();
    const PrimType_ gamma = eulerAngles.yaw();
    const PrimType_ dalpha = eulerAnglesDiff.roll();
    const PrimType_ dbeta = eulerAnglesDiff.pitch();
    const PrimType_ dgamma = eulerAnglesDiff.yaw();
    const PrimType_ t2 = cos(gamma);
    const PrimType_ t3 = cos(beta);
    const PrimType_ t4 = sin(gamma);
    return eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>(dbeta*t4+dalpha*t2*t3, dbeta*t2-dalpha*t3*t4, dgamma+dalpha*sin(beta));
  }
};

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

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

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>& rotationVector, const eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>& angularVelocity) {
    typedef typename eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>::Implementation Vector;
    typedef typename Eigen::Matrix<PrimType_, 3, 3> Matrix3x3;

    // not tested:
//    const Vector rv = rotationVector.toImplementation();
//    const PrimType_ angle = rv.norm();
//
//    if (angle < 1e-14) {
//      return eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>(angularVelocity.toImplementation());
//    }
//
//    const Matrix3x3 rv_hat = linear_algebra::getSkewMatrixFromVector(rv);
//    const Vector rvDiff = (Matrix3x3::Identity() + 0.5*rv_hat + (1.0/(angle*angle) - sin(angle)/(2.0*angle*(1.0-cos(angle))))*rv_hat*rv_hat)*angularVelocity.toImplementation();
//    return eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>(rvDiff);

    const PrimType_ v = rotationVector.vector().norm();
    if (v < 1e-14) {
      return eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>(angularVelocity.toImplementation());
    }
    const PrimType_ w1 = angularVelocity.x();
    const PrimType_ w2 = angularVelocity.y();
    const PrimType_ w3 = angularVelocity.z();

    const PrimType_ v1 = rotationVector.firstEntry();
    const PrimType_ v2 = rotationVector.secondEntry();
    const PrimType_ v3 = rotationVector.thirdEntry();
    const PrimType_ t6 = v*(1.0/2.0);
    const PrimType_ t2 = sin(t6);
    const PrimType_ t3 = t2*t2;
    const PrimType_ t4 = v2*v2;
    const PrimType_ t5 = v*v;
    const PrimType_ t7 = cos(t6);
    const PrimType_ t8 = v1*v1;
    const PrimType_ t9 = v3*v3;
    const PrimType_ t10 = t7*t7;
    const PrimType_ t11 = t3*t3;
    const PrimType_ t12 = t8*t8;
    const PrimType_ t13 = t4*t4;
    const PrimType_ t14 = t9*t9;
    const PrimType_ t15 = t5*t5;
    const PrimType_ t16 = t10*t10;
    const PrimType_ t17 = t2*t11*t12*v;
    const PrimType_ t18 = t2*t11*t13*v;
    const PrimType_ t19 = t2*t11*t14*v;
    const PrimType_ t20 = t3*t7*t10*t15*2.0;
    const PrimType_ t21 = t2*t4*t8*t11*v*2.0;
    const PrimType_ t22 = t2*t8*t9*t11*v*2.0;
    const PrimType_ t23 = t2*t4*t9*t11*v*2.0;
    const PrimType_ t24 = t2*t3*t5*t8*t10*v;
    const PrimType_ t25 = t2*t3*t4*t5*t10*v;
    const PrimType_ t26 = t2*t3*t5*t9*t10*v;
    const PrimType_ t27 = t5*t7*t8*t11*2.0;
    const PrimType_ t28 = t2*t3*t10*t12*v;
    const PrimType_ t29 = t4*t5*t7*t11*2.0;
    const PrimType_ t30 = t2*t5*t8*t16*v;
    const PrimType_ t31 = t2*t3*t10*t13*v;
    const PrimType_ t32 = t5*t7*t9*t11*2.0;
    const PrimType_ t33 = t2*t4*t5*t16*v;
    const PrimType_ t34 = t2*t3*t10*t14*v;
    const PrimType_ t35 = t2*t5*t9*t16*v;
    const PrimType_ t36 = t2*t3*t4*t8*t10*v*2.0;
    const PrimType_ t37 = t2*t3*t8*t9*t10*v*2.0;
    const PrimType_ t38 = t2*t3*t4*t9*t10*v*2.0;
    const PrimType_ t41 = t7*t11*t12*2.0;
    const PrimType_ t42 = t7*t11*t13*2.0;
    const PrimType_ t43 = t7*t11*t14*2.0;
    const PrimType_ t44 = t3*t5*t7*t8*t10*2.0;
    const PrimType_ t45 = t3*t4*t5*t7*t10*2.0;
    const PrimType_ t46 = t3*t5*t7*t9*t10*2.0;
    const PrimType_ t47 = t4*t7*t8*t11*4.0;
    const PrimType_ t48 = t7*t8*t9*t11*4.0;
    const PrimType_ t49 = t4*t7*t9*t11*4.0;
    const PrimType_ t39 = t17+t18+t19+t20+t21+t22+t23+t24+t25+t26+t27+t28+t29+t30+t31+t32+t33+t34+t35+t36+t37+t38-t41-t42-t43-t44-t45-t46-t47-t48-t49;
    const PrimType_ t40 = 1.0/t39;
    const PrimType_ t50 = t9*v*v3*(1.0/2.0);
    const PrimType_ t51 = t8*v*v3*(1.0/2.0);
    const PrimType_ t52 = t4*v*v3*(1.0/2.0);
    const PrimType_ t53 = t9*v*v3;
    const PrimType_ t54 = t8*v*v3;
    const PrimType_ t55 = t4*v*v3;
    const PrimType_ t56 = t8*v3;
    const PrimType_ t57 = t4*v3;
    const PrimType_ t58 = t9*v3;
    const PrimType_ t59 = t7*t10*t15*v1*v2*(1.0/2.0);
    const PrimType_ t60 = t9*v*(1.0/2.0);
    const PrimType_ t61 = t9*v;
    const PrimType_ t62 = t8*v*v1*(1.0/2.0);
    const PrimType_ t63 = t4*v*v1*(1.0/2.0);
    const PrimType_ t64 = t9*v*v1*(1.0/2.0);
    const PrimType_ t65 = t8*v*v1;
    const PrimType_ t66 = t4*v*v1;
    const PrimType_ t67 = t9*v*v1;
    const PrimType_ t68 = t4*v1;
    const PrimType_ t69 = t9*v1;
    const PrimType_ t70 = t8*v1;
    const PrimType_ t71 = t7*t10*t15*v2*v3*(1.0/2.0);
    const PrimType_ t72 = t4*v*v2*(1.0/2.0);
    const PrimType_ t73 = t8*v*v2*(1.0/2.0);
    const PrimType_ t74 = t9*v*v2*(1.0/2.0);
    const PrimType_ t75 = v*v1*v3;
    const PrimType_ t76 = t4*v*v2;
    const PrimType_ t77 = t8*v*v2;
    const PrimType_ t78 = t9*v*v2;
    const PrimType_ t79 = v*v1*v3*2.0;
    const PrimType_ t80 = t8*v2;
    const PrimType_ t81 = t9*v2;
    const PrimType_ t82 = t5*v1*v3*(1.0/2.0);
    const PrimType_ t83 = t4*v2;
    const PrimType_ t84 = t7*t10*t15*v1*v3*(1.0/2.0);
    const PrimType_ t85 = t8*v*(1.0/2.0);
    const PrimType_ t86 = t4*v*(1.0/2.0);
    const PrimType_ t87 = t8*v;
    const PrimType_ t88 = t4*v;
    const PrimType_ dv1 = -t40*w2*(t59+t2*t3*t5*(t50+t51+t52-v*v1*v2)+t2*t5*t10*(t53+t54+t55-v*v1*v2*2.0)*(1.0/2.0)-t3*t5*t7*(t56+t57+t58-t5*v3-t5*v1*v2*(1.0/2.0)))-t40*w3*(t84+t3*t5*t7*(t80+t81+t82+t83-t5*v2)-t2*t3*t5*(t72+t73+t74+t75)-t2*t5*t10*(t76+t77+t78+t79)*(1.0/2.0))+t40*w1*(-t2*t5*t10*v*(t4-t5+t9)+t3*t5*t7*v*(t61+t88)*(1.0/2.0)+t5*t7*t10*v*(t60+t86)+t2*t3*t5*t8*v);
    const PrimType_ dv2 = -t40*w1*(t59-t2*t3*t5*(t50+t51+t52+v*v1*v2)-t2*t5*t10*(t53+t54+t55+v*v1*v2*2.0)*(1.0/2.0)+t3*t5*t7*(t56+t57+t58-t5*v3+t5*v1*v2*(1.0/2.0)))-t40*w3*(t71+t2*t3*t5*(t62+t63+t64-v*v2*v3)+t2*t5*t10*(t65+t66+t67-v*v2*v3*2.0)*(1.0/2.0)-t3*t5*t7*(t68+t69+t70-t5*v1-t5*v2*v3*(1.0/2.0)))+t40*w2*(-t2*t5*t10*v*(-t5+t8+t9)+t3*t5*t7*v*(t61+t87)*(1.0/2.0)+t5*t7*t10*v*(t60+t85)+t2*t3*t4*t5*v);
    const PrimType_ dv3 = -t40*w2*(t71-t2*t3*t5*(t62+t63+t64+v*v2*v3)-t2*t5*t10*(t65+t66+t67+v*v2*v3*2.0)*(1.0/2.0)+t3*t5*t7*(t68+t69+t70-t5*v1+t5*v2*v3*(1.0/2.0)))-t40*w1*(t84+t2*t3*t5*(t72+t73+t74-t75)+t2*t5*t10*(t76+t77+t78-t79)*(1.0/2.0)-t3*t5*t7*(t80+t81-t82+t83-t5*v2))+t40*w3*(-t2*t5*t10*v*(t4-t5+t8)+t3*t5*t7*v*(t87+t88)*(1.0/2.0)+t5*t7*t10*v*(t85+t86)+t2*t3*t5*t9*v);
    return eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>(dv1, dv2, dv3);

  }
};


/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>& rquat, const eigen_impl::LocalAngularVelocity<PrimType_, RotationUsage::ACTIVE>& angularVelocity) {
    Eigen::Matrix<PrimType_,4,3> H_bar_transpose;
    H_bar_transpose << -rquat.toUnitQuaternion().x(), -rquat.toUnitQuaternion().y(), -rquat.toUnitQuaternion().z(),
                        rquat.toUnitQuaternion().w(), -rquat.toUnitQuaternion().z(),  rquat.toUnitQuaternion().y(),
                        rquat.toUnitQuaternion().z(),  rquat.toUnitQuaternion().w(), -rquat.toUnitQuaternion().x(),
                       -rquat.toUnitQuaternion().y(),  rquat.toUnitQuaternion().x(),  rquat.toUnitQuaternion().w();
    return eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>(quaternions::eigen_impl::Quaternion<PrimType_>(0.5*H_bar_transpose*angularVelocity.toImplementation()));
  }
};


template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>& rotationVector, const eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>& rotationVectorDiff) {

    const PrimType_ v = rotationVector.vector().norm();
    if (v < 1e-14) {
      KINDR_ASSERT_TRUE(std::runtime_error, false, "not yet implemented" );
      return eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>();
    }

    const PrimType_ v1 = rotationVector.firstEntry();
    const PrimType_ v2 = rotationVector.secondEntry();
    const PrimType_ v3 = rotationVector.thirdEntry();
    const PrimType_ dv1 = rotationVectorDiff.x();
    const PrimType_ dv2 = rotationVectorDiff.y();
    const PrimType_ dv3 = rotationVectorDiff.z();

    const PrimType_ t2 = 1.0/v;
    const PrimType_ t3 = v*(1.0/2.0);
    const PrimType_ t4 = sin(t3);
    const PrimType_ t5 = 1.0/(v*v*v);
    const PrimType_ t6 = t4*2.0;
    const PrimType_ t7 = cos(t3);
    const PrimType_ t9 = t7*v;
    const PrimType_ t8 = t6-t9;
    const PrimType_ t10 = t2*t4;
    const PrimType_ w = dv1*t2*t4*v1*(-1.0/2.0)-dv2*t2*t4*v2*(1.0/2.0)-dv3*t2*t4*v3*(1.0/2.0);
    const PrimType_ x = dv1*(t10-t5*t8*(v1*v1)*(1.0/2.0))-dv2*t5*t8*v1*v2*(1.0/2.0)-dv3*t5*t8*v1*v3*(1.0/2.0);
    const PrimType_ y = dv2*(t10-t5*t8*(v2*v2)*(1.0/2.0))-dv1*t5*t8*v1*v2*(1.0/2.0)-dv3*t5*t8*v2*v3*(1.0/2.0);
    const PrimType_ z = dv3*(t10-t5*t8*(v3*v3)*(1.0/2.0))-dv1*t5*t8*v1*v3*(1.0/2.0)-dv2*t5*t8*v2*v3*(1.0/2.0);


    return eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>(w, x, y, z);
  }
};

template<typename PrimType_>
class RDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE> convert(const eigen_impl::RotationQuaternion<PrimType_, RotationUsage::ACTIVE>& rquat, const eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>& rotationVectorDiff) {
    return RDiffConversionTraits<eigen_impl::RotationQuaternionDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVectorDiff<PrimType_, RotationUsage::ACTIVE>, eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>>::convert(eigen_impl::RotationVector<PrimType_, RotationUsage::ACTIVE>(rquat), rotationVectorDiff);
  }
};



/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename PrimType_, enum RotationUsage Usage_>
class RDiffConversionTraits<eigen_impl::EulerAnglesXyzDiff<PrimType_, Usage_>, eigen_impl::LocalAngularVelocity<PrimType_, Usage_>, eigen_impl::EulerAnglesXyz<PrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesXyzDiff<PrimType_, Usage_> convert(const eigen_impl::EulerAnglesXyz<PrimType_, Usage_>& eulerAngles, const eigen_impl::LocalAngularVelocity<PrimType_, Usage_>& angularVelocity) {
    typedef typename Eigen::Matrix<PrimType_, 3, 3> Matrix3x3;

    // works:
//    const PrimType_ alpha = eulerAngles.roll();
//    const PrimType_ beta = eulerAngles.pitch();
//    const PrimType_ gamma = eulerAngles.yaw();
//
//    Matrix3x3 H = Matrix3x3::Zero();
//
//    if(cos(beta) == 0) {
//      H << std::numeric_limits<PrimType_>::max(), std::numeric_limits<PrimType_>::max(), 0, sin(gamma), cos(gamma), 0, -cos(gamma)*tan(beta), sin(gamma)*tan(beta), 1;
//    }
//    else {
//      H << cos(gamma)/cos(beta), -sin(gamma)/cos(beta), 0, sin(gamma), cos(gamma), 0, -cos(gamma)*tan(beta), sin(gamma)*tan(beta), 1;
//    }
//
//    return eigen_impl::EulerAnglesXyzDiff<PrimType_, Usage_>(H*angularVelocity.toImplementation());


    const PrimType_ beta_Var = eulerAngles.pitch();
    const PrimType_ gamma_Var = eulerAngles.yaw();
    const PrimType_ w1 = angularVelocity.toImplementation()(0);
    const PrimType_ w2 = angularVelocity.toImplementation()(1);
    const PrimType_ w3 = angularVelocity.toImplementation()(2);
    const PrimType_ t2 = cos(beta_Var);
    PrimType_ t3;
    if (t2 == PrimType_(0)) {
      t3 = std::numeric_limits<PrimType_>::max();
    } else {
      t3 = 1.0/t2;
    }
    const PrimType_ t4 = cos(gamma_Var);
    const PrimType_ t5 = sin(gamma_Var);
    const PrimType_ t6 = sin(beta_Var);
    return eigen_impl::EulerAnglesXyzDiff<PrimType_, Usage_>(t3*t4*w1-t3*t5*w2, t4*w2+t5*w1, w3-t3*t4*t6*w1+t3*t5*t6*w2);

  }
};

/* ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


template<typename PrimType_, enum RotationUsage Usage_>
class RDiffConversionTraits<eigen_impl::EulerAnglesZyxDiff<PrimType_, Usage_>, eigen_impl::LocalAngularVelocity<PrimType_, Usage_>, eigen_impl::EulerAnglesZyx<PrimType_, Usage_>> {
 public:
  inline static eigen_impl::EulerAnglesZyxDiff<PrimType_, Usage_> convert(const eigen_impl::EulerAnglesZyx<PrimType_, Usage_>& eulerAngles, const eigen_impl::LocalAngularVelocity<PrimType_, Usage_>& angularVelocity) {
    typedef typename Eigen::Matrix<PrimType_, 3, 3> Matrix3x3;

    const PrimType_ theta = eulerAngles.pitch();
    const PrimType_ phi = eulerAngles.roll();
    const PrimType_ w1 = angularVelocity.x();
    const PrimType_ w2 = angularVelocity.y();
    const PrimType_ w3 = angularVelocity.z();

    const PrimType_ t2 = cos(theta);
    PrimType_ t3;

    if (t2 == PrimType_(0)) {
      t3 = std::numeric_limits<PrimType_>::max();
    } else {
      t3 = 1.0/t2;
    }

    const PrimType_ t4 = cos(phi);
    const PrimType_ t5 = sin(phi);
    const PrimType_ t6 = sin(theta);
    return eigen_impl::EulerAnglesZyxDiff<PrimType_, Usage_>(t3*t4*w3+t3*t5*w2, t4*w2-t5*w3, w1+t3*t4*t6*w3+t3*t5*t6*w2);

  }
};


} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_RDIFFEIGEN_HPP_ */
