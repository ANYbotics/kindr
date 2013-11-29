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

#ifndef KINDER_ROTATIONEIGEN_HPP_
#define KINDER_ROTATIONEIGEN_HPP_

#include "kinder/common/common.hpp"
#include "kinder/common/assert_macros_eigen.hpp"
#include "kinder/quaternions/QuaternionEigen.hpp"
#include "RotationBase.hpp"
#include "RotationEigenFunctions.hpp"

namespace kinder {
namespace rotations {
//! Implementation of rotations based on the C++ Eigen library
namespace eigen_implementation {

/*! \brief Implementation of an angle axis rotation based on Eigen::AngleAxis
 *  \ingroup rotations
 *  \class AngleAxis
 *  \tparam PrimType the primary type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  The following two typedefs are provided for convenience:
 *   - AngleAxisAD for active rotation and double primary type
 *   - AngleAxisAF for active rotation and float primary type
 *   - AngleAxisPD for passive rotation and double primary type
 *   - AngleAxisPF for passive rotation and float primary type
 */
template<typename PrimType, enum RotationUsage Usage>
class AngleAxis : public AngleAxisBase<AngleAxis<PrimType, Usage>, Usage>, private Eigen::AngleAxis<PrimType> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::AngleAxis<PrimType> Base;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primary type.
   *  Float/Double
   */
  typedef PrimType Scalar;
  /*! \brief The axis type is a 3D vector.
   */
  typedef Eigen::Matrix<PrimType, 3, 1> Vector3;

  /*! \brief Default constructor using identity rotation.
   */
  AngleAxis()
    : Base(Base::Identity()) {
  }

  /*! \brief Constructor using four scalars.
   *  In debug mode, an assertion is thrown if the rotation vector has not unit length.
   *  \param angle     rotation angle
   *  \param v1      first entry of the rotation axis vector
   *  \param v2      second entry of the rotation axis vector
   *  \param v3      third entry of the rotation axis vector
   */
  AngleAxis(const Scalar & angle, const Scalar & v1, const Scalar & v2, const Scalar & v3)
    : Base(angle,Vector3(v1,v2,v3)) {
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using angle and axis.
   * In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param angle   rotation angle
   * \param vector     rotation vector with unit length (Eigen vector)
   */
  AngleAxis(const Scalar & angle, const Vector3 & vector)
    : Base(angle,vector) {
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using Eigen::AngleAxis.
   *  In debug mode, an assertion is thrown if the rotation vector has not unit length.
   *  \param other   Eigen::AngleAxis<PrimType>
   */
  explicit AngleAxis(const Base & other) // explicit on purpose
    : Base(other) {
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input rotation axis has not unit length.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit AngleAxis(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OTHER_DERIVED>
  AngleAxis & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    this->angle() = internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(other.derived()).angle();
    this->axis()  = internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(other.derived()).axis();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  AngleAxis inverted() const {
    return AngleAxis(Base::inverse());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  AngleAxis & invert() {
    *this = AngleAxis(Base::inverse());
    return *this;
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation & toImplementation() {
    return static_cast<Implementation &>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation & toImplementation() const {
    return static_cast<const Implementation &>(*this);
  }

  /*! \brief Reading access to the rotation angle.
   *  \returns rotation angle (scalar) with reading access
   */
  inline Scalar angle() const {
    return Base::angle();
  }

  /*! \brief Reading access to the rotation axis.
   *  \returns rotation axis (vector) with reading access
   */
  inline const Vector3 & axis() const {
    return Base::axis();
  }

  /*! \brief Writing access to the rotation angle.
   *  \returns rotation angle (scalar) with writing access
   */
  inline Scalar & angle() {
    return Base::angle();
  }

  /*! \brief Writing access to the rotation axis.
   *  Attention: No length check in debug mode.
   *  \returns rotation axis (vector) with writing access
   */
  inline Vector3 & axis() {
    return Base::axis();
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  AngleAxis & setIdentity() {
    this->angle() = static_cast<Scalar>(0);
    this->axis() << static_cast<Scalar>(1), static_cast<Scalar>(0), static_cast<Scalar>(0);
    return *this;
  }

  /*! \brief Returns a unique angle axis rotation with angle in [0,pi).
   *  This function is used to compare different rotations.
   *  \returns copy of the angle axis rotation which is unique
   */
  AngleAxis getUnique() const {
    AngleAxis aa(kinder::common::Mod(angle()+M_PI,2*M_PI)-M_PI, axis()); // wraps angle into [-pi,pi)
    if(aa.angle() >= 0)	{
      return aa;
    } else {
      return AngleAxis(-aa.angle(),-aa.axis());
    }
  }

  /*! \brief Modifies the angle axis rotation such that the lies angle in [0,pi).
   *  \returns reference
   */
  AngleAxis & setUnique() {
    AngleAxis aa(kinder::common::Mod(angle()+M_PI,2*M_PI)-M_PI, axis()); // wraps angle into [-pi,pi)
    if(aa.angle() >= 0) { // wraps angle into [0,pi)
      *this = aa;
    } else {
      *this = AngleAxis(-aa.angle(),-aa.axis());
    }
    return *this;
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using AngleAxisBase<AngleAxis<PrimType, Usage>, Usage>::operator*;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const AngleAxis & a) {
    out << a.angle() << ", " << a.axis().transpose();
    return out;
  }
};

//! \brief Active angle axis rotation with double primary type
typedef AngleAxis<double, RotationUsage::ACTIVE>  AngleAxisAD;
//! \brief Active angle axis rotation with float primary type
typedef AngleAxis<float,  RotationUsage::ACTIVE>  AngleAxisAF;
//! \brief Passive angle axis rotation with double primary type
typedef AngleAxis<double, RotationUsage::PASSIVE> AngleAxisPD;
//! \brief Passive angle axis rotation with float primary type
typedef AngleAxis<float,  RotationUsage::PASSIVE> AngleAxisPF;




/*! \brief Implementation of quaternion rotation based on Eigen::Quaternion
 *  \ingroup rotations
 *  \class RotationQuaternion
 *  \tparam PrimType the primary type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  The following two typedefs are provided for convenience:
 *   - RotationQuaternionAD for active rotation and double primary type
 *   - RotationQuaternionAF for active rotation and float primary type
 *   - RotationQuaternionPD for passive rotation and double primary type
 *   - RotationQuaternionPF for passive rotation and float primary type
 */
template<typename PrimType, enum RotationUsage Usage>
class RotationQuaternion : public RotationQuaternionBase<RotationQuaternion<PrimType, Usage>, Usage>, public quaternions::eigen_implementation::UnitQuaternion<PrimType> {
 private:
  /*! \brief The base type.
   */
  typedef quaternions::eigen_implementation::UnitQuaternion<PrimType> Base;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef typename Base::Implementation Implementation;

  /*! \brief The primary type.
   *  Float/Double
   */
  typedef PrimType Scalar;

  /*! \brief Default constructor using identity rotation.
   */
  RotationQuaternion()
    : Base(Implementation::Identity()) {
  }

  /*! \brief Constructor using four scalars.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param w     first entry of the quaternion = cos(phi/2)
   *  \param x     second entry of the quaternion = n1*sin(phi/2)
   *  \param y     third entry of the quaternion = n2*sin(phi/2)
   *  \param z     fourth entry of the quaternion = n3*sin(phi/2)
   */
  RotationQuaternion(const Scalar & w, const Scalar & x, const Scalar & y, const Scalar & z)
    : Base(w,x,y,z) {
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using UnitQuaternion.
   *  \param other   UnitQuaternion
   */
  explicit RotationQuaternion(const Base & other)
    : Base(other) {
  }

  /*! \brief Function for converting the RotationQuaternion back to a UnitQuaternion.
   *  \returns UnitQuaternion
   */
  Base toUnitQuaternion() const {
    return Base(*this);
  }

  /*! \brief Constructor using Eigen::Quaternion<PrimType>.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param other   Eigen::Quaternion<PrimType>
   */
  explicit RotationQuaternion(const Implementation & other)
    : Base(other) {
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit RotationQuaternion(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  /*! \brief Assignment operator using a UnitQuaternion.
   *  \param quat   UnitQuaternion
   *  \returns reference
   */
  template<typename PrimTypeIn>
  RotationQuaternion & operator =(const quaternions::eigen_implementation::UnitQuaternion<PrimTypeIn> & quat) {
    this->w() = quat.w();
    this->x() = quat.x();
    this->y() = quat.y();
    this->z() = quat.z();
    return *this;
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OTHER_DERIVED> // todo: increase efficiency
  RotationQuaternion & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    this->w() = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other.derived()).w();
    this->x() = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other.derived()).x();
    this->y() = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other.derived()).y();
    this->z() = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other.derived()).z();
    return *this;
  }

  /*! \brief Bracket operator which assigns a UnitQuaternion to the RotationQuaternion.
   *  \param quat   UnitQuaternion
   *  \returns reference
   */
  template<typename PrimTypeIn>
  RotationQuaternion & operator ()(const quaternions::eigen_implementation::UnitQuaternion<PrimTypeIn> & quat) {
    this->w() = quat.w();
    this->x() = quat.x();
    this->y() = quat.y();
    this->z() = quat.z();
    return *this;
  }

  /*! \brief Bracket operator which assigns a Quaternion to the RotationQuaternion.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param quat   Quaternion
   *  \returns reference
   */
  template<typename PrimTypeIn>
  RotationQuaternion & operator ()(const quaternions::eigen_implementation::Quaternion<PrimTypeIn> & quat) {
    this->w() = quat.w();
    this->x() = quat.x();
    this->y() = quat.y();
    this->z() = quat.z();
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input quaternion has not unit length.");
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  RotationQuaternion inverted() const {
    return RotationQuaternion(Base::inverted());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  RotationQuaternion & invert() {
    *this = inverted();
    return *this;
  }

  /*! \brief Returns the conjugated of the quaternion.
   *  \returns conjugated of the quaternion
   */
  RotationQuaternion conjugated() const {
    return RotationQuaternion(Base::conjugated());
  }

  /*! \brief Conjugates of the quaternion.
   *  \returns reference
   */
  RotationQuaternion & conjugate() {
    *this = conjugated();
    return *this;
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  RotationQuaternion & setIdentity() {
    this->w() = static_cast<Scalar>(1);
    this->x() = static_cast<Scalar>(0);
    this->y() = static_cast<Scalar>(0);
    this->z() = static_cast<Scalar>(0);
    return *this;
  }

  /*! \brief Returns a unique quaternion rotation with w > 0.
   *  This function is used to compare different rotations.
   *  \returns copy of the quaternion rotation which is unique
   */
  RotationQuaternion getUnique() const {
    if(this->w() >= 0) {
      return *this;
    } else {
      return RotationQuaternion(-this->w(),-this->x(),-this->y(),-this->z());
    }
  }

  /*! \brief Modifies the quaternion rotation such that w >= 0.
   *  \returns reference
   */
  RotationQuaternion & setUnique() {
    if(this->w() < 0) {
      *this = RotationQuaternion(-this->w(),-this->x(),-this->y(),-this->z());
    }
    return *this;
  }

  /*! \brief Returns the norm of the quaternion.
   *  The RotationQuaternion should always have unit length.
   *  \returns norm of the quaternion
   */
  using Base::norm;

  /*! \brief Concenation operator.
   *  This is explicitly specified, because QuaternionBase provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using RotationQuaternionBase<RotationQuaternion<PrimType, Usage>, Usage> ::operator*;

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because QuaternionBase provides also an operator==.
   *  \returns true if two rotations are similar.
   */
  using RotationQuaternionBase<RotationQuaternion<PrimType, Usage>, Usage> ::operator==;
};

//! \brief Active quaternion rotation with double primary type
typedef RotationQuaternion<double, RotationUsage::ACTIVE>  RotationQuaternionAD;
//! \brief Active quaternion rotation with float primary type
typedef RotationQuaternion<float,  RotationUsage::ACTIVE>  RotationQuaternionAF;
//! \brief Passive quaternion rotation with double primary type
typedef RotationQuaternion<double, RotationUsage::PASSIVE> RotationQuaternionPD;
//! \brief Passive quaternion rotation with float primary type
typedef RotationQuaternion<float,  RotationUsage::PASSIVE> RotationQuaternionPF;




/*! \brief Implementation of matrix rotation based on Eigen::Matrix
 *  \ingroup rotations
 *  \class RotationMatrix
 *  \tparam PrimType the primary type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  The following two typedefs are provided for convenience:
 *   - RotationMatrixAD for active rotation and double primary type
 *   - RotationMatrixAF for active rotation and float primary type
 *   - RotationMatrixPD for passive rotation and double primary type
 *   - RotationMatrixPF for passive rotation and float primary type
 */
template<typename PrimType, enum RotationUsage Usage>
class RotationMatrix : public RotationMatrixBase<RotationMatrix<PrimType, Usage>, Usage>, private Eigen::Matrix<PrimType, 3, 3> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType, 3, 3> Base;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primary type.
   *  Float/Double
   */
  typedef PrimType Scalar;

  /*! \brief Default constructor using identity rotation.
   */
  RotationMatrix()
    : Base(Base::Identity()) {
  }

  /*! \brief Constructor using nine scalars.
   *  In debug mode, an assertion is thrown if the matrix is not a rotation matrix.
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
  RotationMatrix(const Scalar & r11, const Scalar & r12, const Scalar & r13,
                 const Scalar & r21, const Scalar & r22, const Scalar & r23,
                 const Scalar & r31, const Scalar & r32, const Scalar & r33) {
    *this << r11,r12,r13,r21,r22,r23,r31,r32,r33;
    KINDER_ASSERT_MATRIX_NEAR_DBG(std::runtime_error, *this * this->transpose(), Base::Identity(), static_cast<Scalar>(1e-6), "Input matrix is not orthogonal.");
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->determinant(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input matrix determinant is not 1.");
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  In debug mode, an assertion is thrown if the rotation vector has not unit length.
   *  \param other   Eigen::Matrix<PrimType,3,3>
   */
  explicit RotationMatrix(const Base & other)
  : Base(other) {
    KINDER_ASSERT_MATRIX_NEAR_DBG(std::runtime_error, other * other.transpose(), Base::Identity(), static_cast<Scalar>(1e-6), "Input matrix is not orthogonal.");
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, other.determinant(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input matrix determinant is not 1.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit RotationMatrix(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<RotationMatrix, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OTHER_DERIVED>
  RotationMatrix & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    *this = internal::ConversionTraits<RotationMatrix, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  RotationMatrix inverted() const {
    return RotationMatrix(toImplementation().transpose());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  RotationMatrix & invert() {
    *this = RotationMatrix(toImplementation().transpose());
    return *this;
  }

  /*! \brief Returns the transpose of the rotation matrix.
   *  \returns the inverse of the rotation
   */
  RotationMatrix transposed() const {
    return RotationMatrix(toImplementation().transpose());
  }

  /*! \brief Transposes the rotation matrix.
   *  \returns reference
   */
  RotationMatrix & transpose() {
    *this = RotationMatrix(toImplementation().transpose());
    return *this;
  }

  /*! \brief Returns the determinant of the rotation matrix.
   *  \returns determinant of the rotation matrix
   */
  Scalar determinant() const {
	return toImplementation().determinant();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation & toImplementation() {
    return static_cast<Implementation &>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation & toImplementation() const {
    return static_cast<const Implementation &>(*this);
  }

  /*! \brief Reading access to the rotation matrix.
   *  \returns rotation matrix (matrix) with reading access
   */
  inline const Implementation & matrix() const {
    return toImplementation();
  }

  /*! \brief Writing access to the rotation matrix.
   *  \returns rotation matrix (matrix) with writing access
   */
  inline Implementation & matrix() {
    return toImplementation();
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  RotationMatrix & setIdentity() {
    this->Implementation::setIdentity();
    return *this;
  }

  /*! \brief Returns a unique matrix rotation.
   *  A rotation matrix is always unique.
   *  This function is used to compare different rotations.
   *  \returns copy of the matrix rotation which is unique
   */
  RotationMatrix getUnique() const {
    return *this;
  }

  /*! \brief Modifies the matrix rotation such that it becomes unique.
   *  A rotation matrix is always unique.
   *  \returns reference
   */
  RotationMatrix & setUnique() {
    return *this;
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using RotationMatrixBase<RotationMatrix<PrimType, Usage>, Usage>::operator*; // otherwise ambiguous RotationBase and Eigen

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator==.
   *  \returns true if two rotations are similar.
   */
  using RotationMatrixBase<RotationMatrix<PrimType, Usage>, Usage>::operator==; // otherwise ambiguous RotationBase and Eigen

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const RotationMatrix & R) {
    out << R.toImplementation();
    return out;
  }
};

//! \brief Active matrix rotation with double primary type
typedef RotationMatrix<double, RotationUsage::ACTIVE>  RotationMatrixAD;
//! \brief Active matrix rotation with float primary type
typedef RotationMatrix<float,  RotationUsage::ACTIVE>  RotationMatrixAF;
//! \brief Passive matrix rotation with double primary type
typedef RotationMatrix<double, RotationUsage::PASSIVE> RotationMatrixPD;
//! \brief Passive matrix rotation with float primary type
typedef RotationMatrix<float,  RotationUsage::PASSIVE> RotationMatrixPF;



/*! \brief Implementation of euler angles (X,Y',Z'' / roll,pitch,yaw) rotation based on Eigen::Matrix
 *  \ingroup rotations
 *  \class EulerAnglesXyz
 *  \tparam PrimType the primary type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  The following two typedefs are provided for convenience:
 *   - EulerAnglesXyzAD for active rotation and double primary type
 *   - EulerAnglesXyzAF for active rotation and float primary type
 *   - EulerAnglesXyzPD for passive rotation and double primary type
 *   - EulerAnglesXyzPF for passive rotation and float primary type
 *   - EulerAnglesRpyAD = EulerAnglesXyzAD
 *   - EulerAnglesRpyAF = EulerAnglesXyzAF
 *   - EulerAnglesRpyPD = EulerAnglesXyzPD
 *   - EulerAnglesRpyPF = EulerAnglesXyzPF
 */
template<typename PrimType, enum RotationUsage Usage>
class EulerAnglesXyz : public EulerAnglesXyzBase<EulerAnglesXyz<PrimType, Usage>, Usage>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primary type.
   *  Float/Double
   */
  typedef PrimType Scalar;

  /*! \brief Default constructor using identity rotation.
   */
  EulerAnglesXyz()
    : Base(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param roll     first rotation angle around X axis
   *  \param pitch    second rotation angle around Y' axis
   *  \param yaw      third rotation angle around Z'' axis
   */
  EulerAnglesXyz(const Scalar & roll, const Scalar & pitch, const Scalar & yaw)
    : Base(roll,pitch,yaw) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType,3,1> [roll; pitch; yaw]
   */
  explicit EulerAnglesXyz(const Base & other)
    : Base(other) {
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit EulerAnglesXyz(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<EulerAnglesXyz, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OTHER_DERIVED>
  EulerAnglesXyz & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    *this = internal::ConversionTraits<EulerAnglesXyz, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  EulerAnglesXyz inverted() const {
    return (EulerAnglesXyz)getInverseRpy<PrimType, PrimType>(*this);
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  EulerAnglesXyz & inverted() {
    *this = (EulerAnglesXyz)getInverseRpy<PrimType, PrimType>(*this);
    return *this;
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  /*! \brief Reading access to roll (X) angle.
   *  \returns roll angle (scalar) with reading access
   */
  inline Scalar roll() const {
    return toImplementation()(0);
  }

  /*! \brief Reading access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with reading access
   */
  inline Scalar pitch() const {
    return toImplementation()(1);
  }

  /*! \brief Reading access to yaw (Z'') angle.
   *  \returns yaw angle (scalar) with reading access
   */
  inline Scalar yaw() const {
    return toImplementation()(2);
  }

  /*! \brief Writing access to roll (X) angle.
   *  \returns roll angle (scalar) with writing access
   */
  inline Scalar & roll() {
    return toImplementation()(0);
  }

  /*! \brief Writing access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with writing access
   */
  inline Scalar & pitch() {
    return toImplementation()(1);
  }

  /*! \brief Writing access to yaw (Z'') angle.
   *  \returns yaw angle (scalar) with writing access
   */
  inline Scalar & yaw() {
    return toImplementation()(2);
  }

  /*! \brief Reading access to roll (X) angle.
   *  \returns roll angle (scalar) with reading access
   */
  inline Scalar x() const {
    return toImplementation()(0);
  }

  /*! \brief Reading access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with reading access
   */
  inline Scalar y() const {
    return toImplementation()(1);
  }

  /*! \brief Reading access to yaw (Z'') angle.
   *  \returns yaw angle (scalar) with reading access
   */
  inline Scalar z() const {
    return toImplementation()(2);
  }

  /*! \brief Writing access to roll (X) angle.
   *  \returns roll angle (scalar) with writing access
   */
  inline Scalar & x() {
    return toImplementation()(0);
  }

  /*! \brief Writing access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with writing access
   */
  inline Scalar & y() {
    return toImplementation()(1);
  }

  /*! \brief Writing access to yaw (Z'') angle.
   *  \returns yaw angle (scalar) with writing access
   */
  inline Scalar & z() {
    return toImplementation()(2);
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  EulerAnglesXyz & setIdentity() {
	  this->Implementation::setZero();
	  return *this;
  }

  /*! \brief Returns a unique euler angles rotation with angles in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  This function is used to compare different rotations.
   *  \returns copy of the euler angles rotation which is unique
   */
  EulerAnglesXyz getUnique() const {
    EulerAnglesXyz xyz(kinder::common::Mod(roll() +M_PI,2*M_PI)-M_PI,
                       kinder::common::Mod(pitch()+M_PI,2*M_PI)-M_PI,
                       kinder::common::Mod(yaw()  +M_PI,2*M_PI)-M_PI); // wraps all angles into [-pi,pi)
    if(xyz.pitch() >= M_PI/2)  // wraps angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    {
      if(xyz.roll() >= 0) {
        xyz.roll() -= M_PI;
      } else {
        xyz.roll() += M_PI;
      }

      xyz.pitch() = -(xyz.pitch()-M_PI);

      if(xyz.yaw() >= 0) {
        xyz.yaw() -= M_PI;
      } else {
        xyz.yaw() += M_PI;
      }
    }
    else if(xyz.pitch() < -M_PI/2)
    {
      if(xyz.roll() >= 0) {
        xyz.roll() -= M_PI;
      } else {
        xyz.roll() += M_PI;
      }

      xyz.pitch() = -(xyz.pitch()+M_PI);

      if(xyz.yaw() >= 0) {
        xyz.yaw() -= M_PI;
      } else {
        xyz.yaw() += M_PI;
      }
    }
    return xyz;
  }

  /*! \brief Modifies the euler angles rotation such that the angles lie in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  \returns reference
   */
  EulerAnglesXyz & setUnique() {
    *this = getUnique();
    return *this;
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using EulerAnglesXyzBase<EulerAnglesXyz<PrimType, Usage>, Usage>::operator*;

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator==.
   *  \returns true if two rotations are similar.
   */
  using EulerAnglesXyzBase<EulerAnglesXyz<PrimType, Usage>, Usage>::operator==;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const EulerAnglesXyz & xyz) {
    out << xyz.toImplementation().transpose();
    return out;
  }
};

//! \brief Active euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primary type
typedef EulerAnglesXyz<double, RotationUsage::ACTIVE>  EulerAnglesXyzAD;
//! \brief Active euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primary type
typedef EulerAnglesXyz<float,  RotationUsage::ACTIVE>  EulerAnglesXyzAF;
//! \brief Passive euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primary type
typedef EulerAnglesXyz<double, RotationUsage::PASSIVE> EulerAnglesXyzPD;
//! \brief Passive euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primary type
typedef EulerAnglesXyz<float,  RotationUsage::PASSIVE> EulerAnglesXyzPF;

//! \brief Equivalent euler angles rotation (X,Y',Z'' / roll,pitch,yaw) class
template <typename PrimType, enum RotationUsage Usage>
using EulerAnglesRpy = EulerAnglesXyz<PrimType, Usage>;

//! \brief Active euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primary type
typedef EulerAnglesRpy<double, RotationUsage::ACTIVE>  EulerAnglesRpyAD;
//! \brief Active euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primary type
typedef EulerAnglesRpy<float,  RotationUsage::ACTIVE>  EulerAnglesRpyAF;
//! \brief Passive euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with double primary type
typedef EulerAnglesRpy<double, RotationUsage::PASSIVE> EulerAnglesRpyPD;
//! \brief Passive euler angles rotation (X,Y',Z'' / roll,pitch,yaw) with float primary type
typedef EulerAnglesRpy<float,  RotationUsage::PASSIVE> EulerAnglesRpyPF;



/*! \brief Implementation of euler angles (Z,Y',X'' / yaw,pitch,roll) rotation based on Eigen::Matrix
 *  \ingroup rotations
 *  \class EulerAnglesZyx
 *  \tparam PrimType the primary type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *  The following two typedefs are provided for convenience:
 *   - EulerAnglesZyxAD for active rotation and double primary type
 *   - EulerAnglesZyxAF for active rotation and float primary type
 *   - EulerAnglesZyxPD for passive rotation and double primary type
 *   - EulerAnglesZyxPF for passive rotation and float primary type
 *   - EulerAnglesYprAD = EulerAnglesZyxAD
 *   - EulerAnglesYprAF = EulerAnglesZyxAF
 *   - EulerAnglesYprPD = EulerAnglesZyxPD
 *   - EulerAnglesYprPF = EulerAnglesZyxPF
 */
template<typename PrimType, enum RotationUsage Usage>
class EulerAnglesZyx : public EulerAnglesZyxBase<EulerAnglesZyx<PrimType, Usage>, Usage>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primary type.
   *  Float/Double
   */
  typedef PrimType Scalar;

  /*! \brief Default constructor using identity rotation.
   */
  EulerAnglesZyx()
    : Base(Base::Zero()) {
  }

  /*! \brief Constructor using three scalars.
   *  \param yaw      first rotation angle around Z axis
   *  \param pitch    second rotation angle around Y' axis
   *  \param roll     third rotation angle around X'' axis
   */
  EulerAnglesZyx(const Scalar & yaw, const Scalar & pitch, const Scalar & roll)
    : Base(yaw,pitch,roll) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType,3,1> [roll; pitch; yaw]
   */
  explicit EulerAnglesZyx(const Base & other)
    : Base(other) {
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit EulerAnglesZyx(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<EulerAnglesZyx, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OTHER_DERIVED>
  EulerAnglesZyx & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    *this = internal::ConversionTraits<EulerAnglesZyx, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  EulerAnglesZyx inverted() const {
    return (EulerAnglesZyx)getInverseRpy<PrimType, PrimType>(*this);
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  EulerAnglesZyx & inverted() {
    *this = (EulerAnglesZyx)getInverseRpy<PrimType, PrimType>(*this);
    return *this;
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  /*! \brief Reading access to yaw (Z) angle.
   *  \returns yaw angle (scalar) with reading access
   */
  inline Scalar yaw() const {
    return toImplementation()(0);
  }

  /*! \brief Reading access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with reading access
   */
  inline Scalar pitch() const {
    return toImplementation()(1);
  }

  /*! \brief Reading access to roll (X'') angle.
   *  \returns roll angle (scalar) with reading access
   */
  inline Scalar roll() const {
    return toImplementation()(2);
  }

  /*! \brief Writing access to yaw (Z) angle.
   *  \returns yaw angle (scalar) with writing access
   */
  inline Scalar & yaw() {
    return toImplementation()(0);
  }

  /*! \brief Writing access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with writing access
   */
  inline Scalar & pitch() {
    return toImplementation()(1);
  }

  /*! \brief Writing access to roll (X'') angle.
   *  \returns roll angle (scalar) with writing access
   */
  inline Scalar & roll() {
    return toImplementation()(2);
  }

  /*! \brief Reading access to yaw (Z) angle.
   *  \returns yaw angle (scalar) with reading access
   */
  inline Scalar z() const {
    return toImplementation()(0);
  }

  /*! \brief Reading access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with reading access
   */
  inline Scalar y() const {
    return toImplementation()(1);
  }

  /*! \brief Reading access to roll (X'') angle.
   *  \returns roll angle (scalar) with reading access
   */
  inline Scalar x() const {
    return toImplementation()(2);
  }

  /*! \brief Writing access to yaw (Z) angle.
   *  \returns yaw angle (scalar) with writing access
   */
  inline Scalar & z() {
    return toImplementation()(0);
  }

  /*! \brief Writing access to pitch (Y') angle.
   *  \returns pitch angle (scalar) with writing access
   */
  inline Scalar & y() {
    return toImplementation()(1);
  }

  /*! \brief Writing access to roll (X'') angle.
   *  \returns roll angle (scalar) with writing access
   */
  inline Scalar & x() {
    return toImplementation()(2);
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  EulerAnglesZyx & setIdentity() {
	  this->Implementation::setZero();
	  return *this;
  }

  /*! \brief Returns a unique euler angles rotation with angles in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  This function is used to compare different rotations.
   *  \returns copy of the euler angles rotation which is unique
   */
  EulerAnglesZyx getUnique() const {  // wraps angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    EulerAnglesZyx zyx(kinder::common::Mod(yaw()  +M_PI,2*M_PI)-M_PI,
                       kinder::common::Mod(pitch()+M_PI,2*M_PI)-M_PI,
                       kinder::common::Mod(roll() +M_PI,2*M_PI)-M_PI); // wraps all angles into [-pi,pi)
    if(zyx.pitch() >= M_PI/2)
    {
      if(zyx.yaw() >= 0) {
        zyx.yaw() -= M_PI;
      } else {
        zyx.yaw() += M_PI;
      }

      zyx.pitch() = -(zyx.pitch()-M_PI);

      if(zyx.roll() >= 0) {
        zyx.roll() -= M_PI;
      } else {
        zyx.roll() += M_PI;
      }
    }
    else if(zyx.pitch() < -M_PI/2)
    {
      if(zyx.yaw() >= 0) {
        zyx.yaw() -= M_PI;
      } else {
        zyx.yaw() += M_PI;
      }

      zyx.pitch() = -(zyx.pitch()+M_PI);

      if(zyx.roll() >= 0) {
        zyx.roll() -= M_PI;
      } else {
        zyx.roll() += M_PI;
      }
    }
    return zyx;
  }

  /*! \brief Modifies the euler angles rotation such that the angles lie in [-pi,pi),[-pi/2,pi/2),[-pi,pi).
   *  \returns reference
   */
  EulerAnglesZyx & setUnique() {  // wraps angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    *this = getUnique();
    return *this;
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using EulerAnglesZyxBase<EulerAnglesZyx<PrimType, Usage>, Usage>::operator*;

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator==.
   *  \returns true if two rotations are similar.
   */
  using EulerAnglesZyxBase<EulerAnglesZyx<PrimType, Usage>, Usage>::operator==;

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const EulerAnglesZyx & zyx) {
    out << zyx.toImplementation().transpose();
    return out;
  }
};

//! \brief Active euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with double primary type
typedef EulerAnglesZyx<double, RotationUsage::ACTIVE>  EulerAnglesZyxAD;
//! \brief Active euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with float primary type
typedef EulerAnglesZyx<float,  RotationUsage::ACTIVE>  EulerAnglesZyxAF;
//! \brief Passive euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with double primary type
typedef EulerAnglesZyx<double, RotationUsage::PASSIVE> EulerAnglesZyxPD;
//! \brief Passive euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with float primary type
typedef EulerAnglesZyx<float,  RotationUsage::PASSIVE> EulerAnglesZyxPF;

//! \brief Equivalent euler angles rotation (Z,Y',X'' / yaw,pitch,roll) class
template <typename PrimType, enum RotationUsage Usage>
using EulerAnglesYpr = EulerAnglesZyx<PrimType, Usage>;

//! \brief Active euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with double primary type
typedef EulerAnglesYpr<double, RotationUsage::ACTIVE>  EulerAnglesYprAD;
//! \brief Active euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with float primary type
typedef EulerAnglesYpr<float,  RotationUsage::ACTIVE>  EulerAnglesYprAF;
//! \brief Passive euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with double primary type
typedef EulerAnglesYpr<double, RotationUsage::PASSIVE> EulerAnglesYprPD;
//! \brief Passive euler angles rotation (Z,Y',X'' / yaw,pitch,roll) with float primary type
typedef EulerAnglesYpr<float,  RotationUsage::PASSIVE> EulerAnglesYprPF;


} // namespace eigen_implementation


namespace internal {


template<typename PrimType, enum RotationUsage Usage>
class get_matrix3X<eigen_implementation::AngleAxis<PrimType, Usage>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType, enum RotationUsage Usage>
class get_matrix3X<eigen_implementation::RotationQuaternion<PrimType, Usage>>{
 public:
  typedef int IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType, enum RotationUsage Usage>
class get_matrix3X<eigen_implementation::RotationMatrix<PrimType, Usage>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType, enum RotationUsage Usage>
class get_matrix3X<eigen_implementation::EulerAnglesXyz<PrimType, Usage>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType, enum RotationUsage Usage>
class get_matrix3X<eigen_implementation::EulerAnglesZyx<PrimType, Usage>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};




template<typename PrimType>
class get_other_usage<eigen_implementation::AngleAxis<PrimType, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_implementation::AngleAxis<PrimType, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::AngleAxis<PrimType, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_implementation::AngleAxis<PrimType, RotationUsage::ACTIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::RotationQuaternion<PrimType, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_implementation::RotationQuaternion<PrimType, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::RotationQuaternion<PrimType, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_implementation::RotationQuaternion<PrimType, RotationUsage::ACTIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::RotationMatrix<PrimType, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_implementation::RotationMatrix<PrimType, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::RotationMatrix<PrimType, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_implementation::RotationMatrix<PrimType, RotationUsage::ACTIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::ACTIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType>
class get_other_usage<eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::ACTIVE> OtherUsage;
};



template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType, Usage>, eigen_implementation::AngleAxis<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::AngleAxis<DestPrimType, Usage> convert(const eigen_implementation::AngleAxis<SourcePrimType, Usage> & a) {
    return eigen_implementation::AngleAxis<DestPrimType, Usage>(a.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType, Usage>, eigen_implementation::RotationQuaternion<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::AngleAxis<DestPrimType, Usage> convert(const eigen_implementation::RotationQuaternion<SourcePrimType, Usage> & q) {
    return eigen_implementation::AngleAxis<DestPrimType, Usage>(eigen_implementation::getAngleAxisFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType, Usage>, eigen_implementation::RotationMatrix<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::AngleAxis<DestPrimType, Usage> convert(const eigen_implementation::RotationMatrix<SourcePrimType, Usage> & R) {
    return eigen_implementation::AngleAxis<DestPrimType, Usage>(eigen_implementation::getAngleAxisFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType, Usage>, eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::AngleAxis<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & xyz) {
    return eigen_implementation::AngleAxis<DestPrimType, Usage>(eigen_implementation::getAngleAxisFromRpy<SourcePrimType, DestPrimType>(xyz.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::AngleAxis<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & zyx) {
    return eigen_implementation::AngleAxis<DestPrimType, Usage>(eigen_implementation::getAngleAxisFromYpr<SourcePrimType, DestPrimType>(zyx.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType, Usage>, eigen_implementation::AngleAxis<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType, Usage> convert(const eigen_implementation::AngleAxis<SourcePrimType, Usage> & aa) {
    return eigen_implementation::RotationQuaternion<DestPrimType, Usage>(eigen_implementation::getQuaternionFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType, Usage>, eigen_implementation::RotationQuaternion<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType, Usage> convert(const eigen_implementation::RotationQuaternion<SourcePrimType, Usage> & q) {
    return eigen_implementation::RotationQuaternion<DestPrimType, Usage>(q.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType, Usage>, eigen_implementation::RotationMatrix<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType, Usage> convert(const eigen_implementation::RotationMatrix<SourcePrimType, Usage> & R) {
    return eigen_implementation::RotationQuaternion<DestPrimType, Usage>(eigen_implementation::getQuaternionFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType, Usage>, eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & xyz) {
    return eigen_implementation::RotationQuaternion<DestPrimType, Usage>(eigen_implementation::getQuaternionFromRpy<SourcePrimType, DestPrimType>(xyz.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & zyx) {
    return eigen_implementation::RotationQuaternion<DestPrimType, Usage>(eigen_implementation::getQuaternionFromYpr<SourcePrimType, DestPrimType>(zyx.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType, Usage>, eigen_implementation::AngleAxis<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationMatrix<DestPrimType, Usage> convert(const eigen_implementation::AngleAxis<SourcePrimType, Usage> & aa) {
    return eigen_implementation::RotationMatrix<DestPrimType, Usage>(eigen_implementation::getRotationMatrixFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType, Usage>, eigen_implementation::RotationQuaternion<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationMatrix<DestPrimType, Usage> convert(const eigen_implementation::RotationQuaternion<SourcePrimType, Usage> & q) {
    return eigen_implementation::RotationMatrix<DestPrimType, Usage>(eigen_implementation::getRotationMatrixFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType, Usage>, eigen_implementation::RotationMatrix<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationMatrix<DestPrimType, Usage> convert(const eigen_implementation::RotationMatrix<SourcePrimType, Usage> & R) {
    return eigen_implementation::RotationMatrix<DestPrimType, Usage>(R.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType, Usage>, eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationMatrix<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & xyz) {
    return eigen_implementation::RotationMatrix<DestPrimType, Usage>(eigen_implementation::getRotationMatrixFromRpy<SourcePrimType, DestPrimType>(xyz.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationMatrix<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & zyx) {
    return eigen_implementation::RotationMatrix<DestPrimType, Usage>(eigen_implementation::getRotationMatrixFromYpr<SourcePrimType, DestPrimType>(zyx.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>, eigen_implementation::AngleAxis<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesXyz<DestPrimType, Usage> convert(const eigen_implementation::AngleAxis<SourcePrimType, Usage> & aa) {
    return eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>(eigen_implementation::getRpyFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>, eigen_implementation::RotationQuaternion<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesXyz<DestPrimType, Usage> convert(const eigen_implementation::RotationQuaternion<SourcePrimType, Usage> & q) {
    return eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>(eigen_implementation::getRpyFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>, eigen_implementation::RotationMatrix<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesXyz<DestPrimType, Usage> convert(const eigen_implementation::RotationMatrix<SourcePrimType, Usage> & R) {
    return eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>(eigen_implementation::getRpyFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>, eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesXyz<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & xyz) {
    return eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>(xyz.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesXyz<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & zyx) {
    return eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>(eigen_implementation::getRpyFromYpr<SourcePrimType, DestPrimType>(zyx.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>, eigen_implementation::AngleAxis<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesZyx<DestPrimType, Usage> convert(const eigen_implementation::AngleAxis<SourcePrimType, Usage> & aa) {
    return eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>(eigen_implementation::getYprFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>, eigen_implementation::RotationQuaternion<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesZyx<DestPrimType, Usage> convert(const eigen_implementation::RotationQuaternion<SourcePrimType, Usage> & q) {
    return eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>(eigen_implementation::getYprFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>, eigen_implementation::RotationMatrix<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesZyx<DestPrimType, Usage> convert(const eigen_implementation::RotationMatrix<SourcePrimType, Usage> & R) {
    return eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>(eigen_implementation::getYprFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>, eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesZyx<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & xyz) {
    return eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>(eigen_implementation::getYprFromRpy<SourcePrimType, DestPrimType>(xyz.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesZyx<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & zyx) {
    return eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>(zyx.toImplementation().template cast<DestPrimType>());
  }
};




template<typename PrimType>
class MultiplicationTraits<RotationBase<eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::ACTIVE>, RotationUsage::ACTIVE>, RotationBase<eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::ACTIVE>, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::ACTIVE> mult(const eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::ACTIVE> & a, const eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::ACTIVE> & b) {
    return eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::ACTIVE>(eigen_implementation::RotationQuaternion<PrimType, RotationUsage::ACTIVE>(
                                                                 eigen_implementation::RotationQuaternion<PrimType, RotationUsage::ACTIVE>(a).toImplementation()*
                                                                 eigen_implementation::RotationQuaternion<PrimType, RotationUsage::ACTIVE>(b).toImplementation()));
  }
};

template<typename PrimType>
class MultiplicationTraits<RotationBase<eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::PASSIVE>, RotationUsage::PASSIVE>, RotationBase<eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::PASSIVE>, RotationUsage::PASSIVE>> {
 public:
  inline static eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::PASSIVE> mult(const eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::PASSIVE> & a, const eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::PASSIVE> & b) {
    return eigen_implementation::EulerAnglesXyz<PrimType, RotationUsage::PASSIVE>(eigen_implementation::RotationQuaternion<PrimType, RotationUsage::PASSIVE>(
                                                                 eigen_implementation::RotationQuaternion<PrimType, RotationUsage::PASSIVE>(a).toImplementation()*
                                                                 eigen_implementation::RotationQuaternion<PrimType, RotationUsage::PASSIVE>(b).toImplementation()));
  }
};

template<typename PrimType>
class MultiplicationTraits<RotationBase<eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::ACTIVE>, RotationUsage::ACTIVE>, RotationBase<eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::ACTIVE>, RotationUsage::ACTIVE>> {
 public:
  inline static eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::ACTIVE> mult(const eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::ACTIVE> & a, const eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::ACTIVE> & b) {
    return eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::ACTIVE>(eigen_implementation::RotationQuaternion<PrimType, RotationUsage::ACTIVE>(
                                                                 eigen_implementation::RotationQuaternion<PrimType, RotationUsage::ACTIVE>(a).toImplementation()*
                                                                 eigen_implementation::RotationQuaternion<PrimType, RotationUsage::ACTIVE>(b).toImplementation()));
  }
};

template<typename PrimType>
class MultiplicationTraits<RotationBase<eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::PASSIVE>, RotationUsage::PASSIVE>, RotationBase<eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::PASSIVE>, RotationUsage::PASSIVE>> {
 public:
  inline static eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::PASSIVE> mult(const eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::PASSIVE> & a, const eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::PASSIVE> & b) {
    return eigen_implementation::EulerAnglesZyx<PrimType, RotationUsage::PASSIVE>(eigen_implementation::RotationQuaternion<PrimType, RotationUsage::PASSIVE>(
                                                                 eigen_implementation::RotationQuaternion<PrimType, RotationUsage::PASSIVE>(a).toImplementation()*
                                                                 eigen_implementation::RotationQuaternion<PrimType, RotationUsage::PASSIVE>(b).toImplementation()));
  }
};


//template<typename PrimType, enum RotationUsage Usage>
//class MultiplicationTraits<RotationBase<eigen_implementation::EulerAnglesXyz<PrimType, Usage>, Usage>, RotationBase<eigen_implementation::EulerAnglesXyz<PrimType, Usage>, Usage>> {
// public:
//  inline static eigen_implementation::EulerAnglesXyz<PrimType, Usage> mult(const eigen_implementation::EulerAnglesXyz<PrimType, Usage> & a, const eigen_implementation::EulerAnglesXyz<PrimType, Usage> & b) {
//    return eigen_implementation::EulerAnglesXyz<PrimType, Usage>(eigen_implementation::RotationQuaternion<PrimType, Usage>(
//                                                                 eigen_implementation::RotationQuaternion<PrimType, Usage>(a).toImplementation()*
//                                                                 eigen_implementation::RotationQuaternion<PrimType, Usage>(b).toImplementation()));
//  }
//};
//
//template<typename PrimType, enum RotationUsage Usage>
//class MultiplicationTraits<RotationBase<eigen_implementation::EulerAnglesZyx<PrimType, Usage>, Usage>, RotationBase<eigen_implementation::EulerAnglesZyx<PrimType, Usage>, Usage>> {
// public:
//  inline static eigen_implementation::EulerAnglesZyx<PrimType, Usage> mult(const eigen_implementation::EulerAnglesZyx<PrimType, Usage> & a, const eigen_implementation::EulerAnglesZyx<PrimType, Usage> & b) {
//    return eigen_implementation::EulerAnglesZyx<PrimType, Usage>(eigen_implementation::RotationQuaternion<PrimType, Usage>(
//                                                                 eigen_implementation::RotationQuaternion<PrimType, Usage>(a).toImplementation()*
//                                                                 eigen_implementation::RotationQuaternion<PrimType, Usage>(b).toImplementation()));
//  }
//};



template<typename PrimType, enum RotationUsage Usage>
class RotationTraits<eigen_implementation::AngleAxis<PrimType, Usage>> {
 public:
  template<typename get_matrix3X<eigen_implementation::AngleAxis<PrimType, Usage>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_implementation::AngleAxis<PrimType, Usage>>::template Matrix3X<Cols> rotate(const eigen_implementation::AngleAxis<PrimType, Usage> & aa, const typename get_matrix3X<eigen_implementation::AngleAxis<PrimType, Usage>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType, Usage>(aa).toImplementation() * m;
  }
};

template<typename PrimType, enum RotationUsage Usage>
class RotationTraits<eigen_implementation::RotationQuaternion<PrimType, Usage>> {
 public:
  template<typename get_matrix3X<eigen_implementation::RotationQuaternion<PrimType, Usage>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_implementation::RotationQuaternion<PrimType, Usage>>::template Matrix3X<Cols> rotate(const eigen_implementation::RotationQuaternion<PrimType, Usage> & p, const typename get_matrix3X<eigen_implementation::RotationQuaternion<PrimType, Usage>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType, Usage>(p).toImplementation() * m;
  }
};

template<typename PrimType, enum RotationUsage Usage>
class RotationTraits<eigen_implementation::RotationMatrix<PrimType, Usage>> {
 public:
  template<typename get_matrix3X<eigen_implementation::RotationMatrix<PrimType, Usage>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_implementation::RotationMatrix<PrimType, Usage>>::template Matrix3X<Cols> rotate(const eigen_implementation::RotationMatrix<PrimType, Usage> & R, const typename get_matrix3X<eigen_implementation::RotationMatrix<PrimType, Usage>>::template Matrix3X<Cols> & m){
    return R.toImplementation() * m;
  }
};

template<typename PrimType, enum RotationUsage Usage>
class RotationTraits<eigen_implementation::EulerAnglesXyz<PrimType, Usage>> {
 public:
  template<typename get_matrix3X<eigen_implementation::EulerAnglesXyz<PrimType, Usage>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_implementation::EulerAnglesXyz<PrimType, Usage>>::template Matrix3X<Cols> rotate(const eigen_implementation::EulerAnglesXyz<PrimType, Usage> & xyz, const typename get_matrix3X<eigen_implementation::EulerAnglesXyz<PrimType, Usage>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType, Usage>(xyz).toImplementation() * m;
  }
};

template<typename PrimType, enum RotationUsage Usage>
class RotationTraits<eigen_implementation::EulerAnglesZyx<PrimType, Usage>> {
 public:
  template<typename get_matrix3X<eigen_implementation::EulerAnglesZyx<PrimType, Usage>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_implementation::EulerAnglesZyx<PrimType, Usage>>::template Matrix3X<Cols> rotate(const eigen_implementation::EulerAnglesZyx<PrimType, Usage> & zyx, const typename get_matrix3X<eigen_implementation::EulerAnglesZyx<PrimType, Usage>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType, Usage>(zyx).toImplementation() * m;
  }
};



template<typename PrimType, enum RotationUsage Usage>
class ComparisonTraits<eigen_implementation::AngleAxis<PrimType, Usage>> {
 public:
  inline static bool isEqual(const eigen_implementation::AngleAxis<PrimType, Usage> & a, const eigen_implementation::AngleAxis<PrimType, Usage> & b){
    return a.toImplementation().angle() ==  b.toImplementation().angle() &&
           a.toImplementation().axis()  ==  b.toImplementation().axis();
  }
};

template<typename PrimType, enum RotationUsage Usage>
class ComparisonTraits<eigen_implementation::RotationQuaternion<PrimType, Usage>> {
 public:
   inline static bool isEqual(const eigen_implementation::RotationQuaternion<PrimType, Usage> & a, const eigen_implementation::RotationQuaternion<PrimType, Usage> & b){
     return a.toImplementation().w() ==  b.toImplementation().w() &&
            a.toImplementation().x() ==  b.toImplementation().x() &&
            a.toImplementation().y() ==  b.toImplementation().y() &&
            a.toImplementation().z() ==  b.toImplementation().z();
   }

   inline static bool isNear(const eigen_implementation::RotationQuaternion<PrimType, Usage> & a, const eigen_implementation::RotationQuaternion<PrimType, Usage> & b, PrimType tol){
     return abs(a.toImplementation().w() - b.toImplementation().w() < tol) &&
    		    abs(a.toImplementation().x() - b.toImplementation().x() < tol) &&
    	    	abs(a.toImplementation().y() - b.toImplementation().y() < tol) &&
    	    	abs(a.toImplementation().z() - b.toImplementation().z() < tol);
   }
};



} // namespace internal
} // namespace rotations
} // namespace rm

#endif /* KINDER_ROTATIONEIGEN_HPP_ */
