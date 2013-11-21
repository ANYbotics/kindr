/*
 * RotationEigen.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef ROTATIONEIGEN_HPP_
#define ROTATIONEIGEN_HPP_

#include "rm/common/common.hpp"
#include "rm/common/assert_macros_eigen.hpp"
#include "rm/quaternions/QuaternionEigen.hpp"
#include "RotationBase.hpp"
#include "RotationEigenFunctions.hpp"

namespace rm {
namespace rotations {
//! Implementation of rotations based on the C++ Eigen library
namespace eigen_implementation {

//!  Angle-axis rotation
template<typename PrimType, enum RotationUsage Usage>
class AngleAxis : public AngleAxisBase<AngleAxis<PrimType, Usage>, Usage>, private Eigen::AngleAxis<PrimType> {
 private:
  //! the base type, i.e., Eigen::AngleAxis
  typedef Eigen::AngleAxis<PrimType> Base;
 public:
  //! the implementation type, i.e., Eigen::AngleAxis
  typedef Base Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;
  //! the type of a 3D vector
  typedef Eigen::Matrix<PrimType, 3, 1> Vector3;

  //! Default constructor initializes the angle-axis rotation to the identity rotation
  AngleAxis()
    : Base(Base::Identity()) {
  }

  //! Constructor initializes the angle and the vector
  /*!
   * In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param chi     rotation angle
   * \param v1      first entry of rotation axis vector
   * \param v2      second entry of rotation axis vector
   * \param v3      third entry of rotation axis vector
   */
  AngleAxis(const Scalar & chi, const Scalar & v1, const Scalar & v2, const Scalar & v3)
    : Base(chi,Vector3(v1,v2,v3)) {
    RM_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input rotation axis has not unit length.");
  }

  //! Constructor initializes the angle and the vector from an Eigen::Vector3
  /*!
   * In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param chi   rotation angle
   * \param v     rotation vector with unit length
   */
  AngleAxis(const Scalar & chi, const Vector3 & v)
    : Base(chi,v) {
    RM_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input rotation axis has not unit length.");
  }

  //! Constructor initializes from Eigen::AngleAxis
  /*! In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param other   Eigen::AngleAxis
   */
  explicit AngleAxis(const Base & other)
    : Base(other) {
    RM_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->axis().norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input rotation axis has not unit length.");
  }

  //! Constructor initializes from another rotation
  /*!
   * \param other   other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit AngleAxis(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  AngleAxis & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    this->angle() = internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(other.derived()).angle();
    this->axis()  = internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(other.derived()).axis();
    return *this;
  }

  /*! \returns the inverse of the rotation
   */
  AngleAxis inverted() const {
    return AngleAxis(Base::inverse());
  }

  /*! \inverts the rotation
   */
  AngleAxis & invert() {
    *this = AngleAxis(Base::inverse());
    return *this;
  }

  /*! \returns the implementation for direct manipulation (only for advanced users recommended)
   */
  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }

  /*! \returns the implementation
   */
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using AngleAxisBase<AngleAxis<PrimType, Usage>, Usage>::operator*; // otherwise ambiguous RotationBase and Eigen

  /*! \returns the rotation angle
   */
  inline Scalar angle() const {
    return Base::angle();
  }

  /*! \returns the vector representing the rotation axis with unit length
   */
  inline const Vector3 & axis() const {
    return Base::axis();
  }

  /*! \returns the rotation angle for manipulation
    */
  inline Scalar & angle() {
    return Base::angle();
  }

  /*! Note that the vector needs to have unit length!
   * \returns the vector representing the rotation axis with unit length for manipulation
   */
  inline Vector3 & axis() {
    return Base::axis();
  }

  AngleAxis & setIdentity() {
	this->angle() = static_cast<Scalar>(0);
	this->axis() << static_cast<Scalar>(1), static_cast<Scalar>(0), static_cast<Scalar>(0);
	return *this;
  }

  AngleAxis getUnique() const {
	AngleAxis aa(rm::common::Mod(angle()+M_PI,2*M_PI)-M_PI, axis()); // wraps angle into [-pi,pi)
	if(aa.angle() >= 0)	{
		return aa;
	} else {
		return AngleAxis(-aa.angle(),-aa.axis());
	}
  }

  friend std::ostream & operator << (std::ostream & out, const AngleAxis & a) {
    out << a.angle() << ", " << a.axis().transpose();
    return out;
  }
};
//! Angle-axis rotation with double
typedef AngleAxis<double, RotationUsage::ACTIVE>  AngleAxisAD;
//! Angle-axis rotation with float
typedef AngleAxis<float,  RotationUsage::ACTIVE>  AngleAxisAF;
//! Angle-axis rotation with double
typedef AngleAxis<double, RotationUsage::PASSIVE> AngleAxisPD;
//! Angle-axis rotation with float
typedef AngleAxis<float,  RotationUsage::PASSIVE> AngleAxisPF;



//! Quaternion rotation
template<typename PrimType, enum RotationUsage Usage>
class RotationQuaternion : public RotationQuaternionBase<RotationQuaternion<PrimType, Usage>, Usage>, public quaternions::eigen_implementation::UnitQuaternion<PrimType> {
 private:
  //! the base type, i.e., quaternions::eigen_implementation::UnitQuaternion
  typedef quaternions::eigen_implementation::UnitQuaternion<PrimType> Base;
 public:
  //! the implementation type, i.e., Eigen::Quaternion
  typedef typename Base::Implementation Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;

  RotationQuaternion()
    : Base(Implementation::Identity()) {
  }

  RotationQuaternion(const Scalar & w, const Scalar & x, const Scalar & y, const Scalar & z)
    : Base(w,x,y,z) {
    RM_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input quaternion has not unit length.");
  }

//  wWithRealScalarGreaterOrEqualZero();
//  xWithRealScalarGreaterOrEqualZero();
//  yWithRealScalarGreaterOrEqualZero();

//  const quaternions::eigen_implementation::UnitQuaternion<PrimType, Usage> & getUnitQuaternion() const;
//  quaternions::eigen_implementation::UnitQuaternion<PrimType, Usage> getUnitQuaternionWithRealScalarGreaterOrEqualZero() const;

  // create from UnitQuaternion
  explicit RotationQuaternion(const Base & other)
    : Base(other) {
  }

  Base toUnitQuaternion() const {
    return Base(*this);
  }

  // create from Eigen::Quaternion
  explicit RotationQuaternion(const Implementation & other)
    : Base(other) {
    RM_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input quaternion has not unit length.");
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit RotationQuaternion(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename PrimTypeIn>
  RotationQuaternion & operator =(const quaternions::eigen_implementation::UnitQuaternion<PrimTypeIn> & quat) {
    this->w() = quat.w();
    this->x() = quat.x();
    this->y() = quat.y();
    this->z() = quat.z();
    return *this;
  }

  template<typename OTHER_DERIVED>
  RotationQuaternion & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
//    *this = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
//    RotationQuaternion result = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
//    this->w() = result.w();
//    this->x() = result.x();
//    this->y() = result.y();
//    this->z() = result.z();

    this->w() = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other.derived()).w();
    this->x() = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other.derived()).x();
    this->y() = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other.derived()).y();
    this->z() = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other.derived()).z();
    return *this;
  }

  template<typename PrimTypeIn>
  RotationQuaternion & operator ()(const quaternions::eigen_implementation::UnitQuaternion<PrimTypeIn> & quat) {
    this->w() = quat.w();
    this->x() = quat.x();
    this->y() = quat.y();
    this->z() = quat.z();
    return *this;
  }

  template<typename PrimTypeIn>
  RotationQuaternion & operator ()(const quaternions::eigen_implementation::Quaternion<PrimTypeIn> & quat) {
    this->w() = quat.w();
    this->x() = quat.x();
    this->y() = quat.y();
    this->z() = quat.z();
    RM_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input quaternion has not unit length.");
    return *this;
  }

  /*! \returns the inverse of the rotation
   */
  RotationQuaternion inverted() const {
    return RotationQuaternion(Base::inverted());
  }

  /*! \inverts the rotation
   */
  RotationQuaternion & invert() {
    *this = inverted();
    return *this;
  }

  /*! \returns the conjugated of the rotation
   */
  RotationQuaternion conjugated() const {
    return RotationQuaternion(Base::conjugated());
  }

  /*! \conjugates the rotation
   */
  RotationQuaternion & conjugate() {
    *this = conjugated();
    return *this;
  }

  RotationQuaternion & setIdentity() {
//	  this->Implementation::setIdentity(); // inaccessible
    this->w() = static_cast<Scalar>(1); // todo
    this->x() = static_cast<Scalar>(0);
    this->y() = static_cast<Scalar>(0);
    this->z() = static_cast<Scalar>(0);
    return *this;
  }

  RotationQuaternion getUnique() const {
    if(this->w() >= 0) {
      return *this;
    } else {
      return RotationQuaternion(-this->w(),-this->x(),-this->y(),-this->z());
    }
  }

  using Base::norm;
  using Base::toImplementation;

//  using Base::operator *;
  using RotationQuaternionBase<RotationQuaternion<PrimType, Usage>, Usage> ::operator *;
  using RotationQuaternionBase<RotationQuaternion<PrimType, Usage>, Usage> ::operator ==;




//  bool operator ==(const quaternions::eigen_implementation::UnitQuaternion<PrimType> & other) {
//    return Base::operator == (static_cast<const Base &>(other));
//  }

//  RotationQuaternion operator *(const quaternions::eigen_implementation::Quaternion<PrimType> & other) const {
//	  return RotationQuaternion(Base::operator*(other));
//  }

//  using Base::operator ==;
//  using RotationQuaternionBase<RotationQuaternion<PrimType, Usage>>::operator*; // otherwise ambiguous RotationBase and QuaternionBase
//  using RotationQuaternionBase<RotationQuaternion<PrimType, Usage>>::operator==; // otherwise ambiguous RotationBase and Eigen
};

typedef RotationQuaternion<double, RotationUsage::ACTIVE>  RotationQuaternionAD;
typedef RotationQuaternion<float,  RotationUsage::ACTIVE>  RotationQuaternionAF;
typedef RotationQuaternion<double, RotationUsage::PASSIVE> RotationQuaternionPD;
typedef RotationQuaternion<float,  RotationUsage::PASSIVE> RotationQuaternionPF;



//! Rotation matrix
template<typename PrimType, enum RotationUsage Usage>
class RotationMatrix : public RotationMatrixBase<RotationMatrix<PrimType, Usage>, Usage>, private Eigen::Matrix<PrimType, 3, 3> {
 private:
  //! the base type, i.e., Eigen::Matrix
  typedef Eigen::Matrix<PrimType, 3, 3> Base;
 public:
  //! the implementation type, i.e., Eigen::Matrix
  typedef Base Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;

  RotationMatrix()
    : Base(Base::Identity()) {
  }

  RotationMatrix(const Scalar & r11, const Scalar & r12, const Scalar & r13,
                 const Scalar & r21, const Scalar & r22, const Scalar & r23,
                 const Scalar & r31, const Scalar & r32, const Scalar & r33) {
    *this << r11,r12,r13,r21,r22,r23,r31,r32,r33;
    RM_ASSERT_MATRIX_NEAR_DBG(std::runtime_error, *this * this->transpose(), Base::Identity(), static_cast<Scalar>(1e-6), "Input matrix is not orthogonal.");
    RM_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->determinant(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input matrix determinant is not 1.");
  }

  // create from Eigen::Matrix
  explicit RotationMatrix(const Base & other)
    : Base(other) {
	RM_ASSERT_MATRIX_NEAR_DBG(std::runtime_error, other * other.transpose(), Base::Identity(), static_cast<Scalar>(1e-6), "Input matrix is not orthogonal.");
	RM_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, other.determinant(), static_cast<Scalar>(1), static_cast<Scalar>(1e-6), "Input matrix determinant is not 1.");
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit RotationMatrix(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<RotationMatrix, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  RotationMatrix & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    *this = internal::ConversionTraits<RotationMatrix, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  /*! \returns the inverse of the rotation
   */
  RotationMatrix inverted() const {
    return RotationMatrix(toImplementation().transpose());
  }

  /*! \inverts the rotation
   */
  RotationMatrix & invert() {
    *this = RotationMatrix(toImplementation().transpose());
    return *this;
  }

  /*! \returns the transpose of the rotation matrix
   */
  RotationMatrix transposed() const {
    return RotationMatrix(toImplementation().transpose());
  }

  /*! \transposes the rotation matrix
   */
  RotationMatrix & transpose() {
    *this = RotationMatrix(toImplementation().transpose());
    return *this;
  }

  Scalar determinant() const {
	return toImplementation().determinant();
  }

  inline Implementation & toImplementation() {
    return static_cast<Implementation &>(*this);
  }
  inline const Implementation & toImplementation() const {
    return static_cast<const Implementation &>(*this);
  }

  using RotationMatrixBase<RotationMatrix<PrimType, Usage>, Usage>::operator*; // otherwise ambiguous RotationBase and Eigen
  using RotationMatrixBase<RotationMatrix<PrimType, Usage>, Usage>::operator==; // otherwise ambiguous RotationBase and Eigen

  inline const Implementation & matrix() const {
    return toImplementation();
  }

  inline Implementation & matrix() {
    return toImplementation();
  }

  RotationMatrix & setIdentity() {
	this->Implementation::setIdentity();
	return *this;
  }

  RotationMatrix getUnique() const {
	return *this;
  }

  friend std::ostream & operator << (std::ostream & out, const RotationMatrix & R) {
    out << R.toImplementation();
    return out;
  }
};

typedef RotationMatrix<double, RotationUsage::ACTIVE>  RotationMatrixAD;
typedef RotationMatrix<float,  RotationUsage::ACTIVE>  RotationMatrixAF;
typedef RotationMatrix<double, RotationUsage::PASSIVE> RotationMatrixPD;
typedef RotationMatrix<float,  RotationUsage::PASSIVE> RotationMatrixPF;



//! Euler angles with X-Y''-Z''' convention
template<typename PrimType, enum RotationUsage Usage>
class EulerAnglesXyz : public EulerAnglesXyzBase<EulerAnglesXyz<PrimType, Usage>, Usage>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  //! the base type, i.e., Eigen::Matrix
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  //! the implementation type, i.e., Eigen::Matrix
  typedef Base Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;

  EulerAnglesXyz()
    : Base(Base::Zero()) {
  }

  EulerAnglesXyz(const Scalar & r, const Scalar & p, const Scalar & y)
    : Base(r,p,y) {
  }

  // create from Eigen::Matrix
  explicit EulerAnglesXyz(const Base & other)
    : Base(other) {
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit EulerAnglesXyz(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<EulerAnglesXyz, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  EulerAnglesXyz & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    *this = internal::ConversionTraits<EulerAnglesXyz, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  /*! \returns the inverse of the rotation
   */
  EulerAnglesXyz inverted() const {
    return (EulerAnglesXyz)getInverseRpy<PrimType, PrimType>(*this);
  }

  /*! \inverts the rotation
   */
  EulerAnglesXyz & inverted() {
    *this = (EulerAnglesXyz)getInverseRpy<PrimType, PrimType>(*this);
    return *this;
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using EulerAnglesXyzBase<EulerAnglesXyz<PrimType, Usage>, Usage>::operator*; // otherwise ambiguous RotationBase and Eigen
  using EulerAnglesXyzBase<EulerAnglesXyz<PrimType, Usage>, Usage>::operator==; // otherwise ambiguous RotationBase and Eigen

  inline Scalar roll() const {
    return toImplementation()(0);
  }

  inline Scalar pitch() const {
    return toImplementation()(1);
  }

  inline Scalar yaw() const {
    return toImplementation()(2);
  }

  inline Scalar & roll() {
    return toImplementation()(0);
  }

  inline Scalar & pitch() {
    return toImplementation()(1);
  }

  inline Scalar & yaw() {
    return toImplementation()(2);
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

  inline Scalar & x() {
    return toImplementation()(0);
  }

  inline Scalar & y() {
    return toImplementation()(1);
  }

  inline Scalar & z() {
    return toImplementation()(2);
  }

  EulerAnglesXyz & setIdentity() {
	  this->Implementation::setZero();
	  return *this;
  }

  EulerAnglesXyz getUnique() const {  // wraps angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    EulerAnglesXyz rpy(rm::common::Mod(roll() +M_PI,2*M_PI)-M_PI,
                       rm::common::Mod(pitch()+M_PI,2*M_PI)-M_PI,
                       rm::common::Mod(yaw()  +M_PI,2*M_PI)-M_PI); // wraps all angles into [-pi,pi)
    if(rpy.pitch() >= M_PI/2)
    {
      if(rpy.roll() >= 0) {
        rpy.roll() -= M_PI;
      } else {
        rpy.roll() += M_PI;
      }

      rpy.pitch() = -(rpy.pitch()-M_PI);

      if(rpy.yaw() >= 0) {
        rpy.yaw() -= M_PI;
      } else {
        rpy.yaw() += M_PI;
      }
    }
    else
      if(rpy.pitch() < -M_PI/2)
      {
        if(rpy.roll() >= 0) {
          rpy.roll() -= M_PI;
        } else {
          rpy.roll() += M_PI;
        }

        rpy.pitch() = -(rpy.pitch()+M_PI);

        if(rpy.yaw() >= 0) {
          rpy.yaw() -= M_PI;
        } else {
          rpy.yaw() += M_PI;
        }
      }
    return rpy;
  }

  friend std::ostream & operator << (std::ostream & out, const EulerAnglesXyz & rpy) {
    out << rpy.toImplementation().transpose();
    return out;
  }
};

typedef EulerAnglesXyz<double, RotationUsage::ACTIVE>  EulerAnglesXyzAD;
typedef EulerAnglesXyz<float,  RotationUsage::ACTIVE>  EulerAnglesXyzAF;
typedef EulerAnglesXyz<double, RotationUsage::PASSIVE> EulerAnglesXyzPD;
typedef EulerAnglesXyz<float,  RotationUsage::PASSIVE> EulerAnglesXyzPF;

template <typename PrimType, enum RotationUsage Usage>
using EulerAnglesRpy = EulerAnglesXyz<PrimType, Usage>;

typedef EulerAnglesRpy<double, RotationUsage::ACTIVE>  EulerAnglesRpyAD;
typedef EulerAnglesRpy<float,  RotationUsage::ACTIVE>  EulerAnglesRpyAF;
typedef EulerAnglesRpy<double, RotationUsage::PASSIVE> EulerAnglesRpyPD;
typedef EulerAnglesRpy<float,  RotationUsage::PASSIVE> EulerAnglesRpyPF;



//! Euler angles with Z-Y''-X''' convention
template<typename PrimType, enum RotationUsage Usage>
class EulerAnglesZyx : public EulerAnglesZyxBase<EulerAnglesZyx<PrimType, Usage>, Usage>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  //! the base type, i.e., Eigen::Matrix
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  //! the implementation type, i.e., Eigen::Matrix
  typedef Base Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;

  //! Default constructor initializes all angles with zero
  EulerAnglesZyx()
    : Base(Base::Zero()) {
  }

  //! Constructor initializes all three angles
  /*!
   * \param yaw     yaw angle
   * \param pitch   pitch angle
   * \param roll    roll angle
   */
  EulerAnglesZyx(const Scalar & yaw, const Scalar & pitch, const Scalar & roll)
    : Base(yaw,pitch,roll) {
  }

  //! Creates from Eigen::Matrix
  /*!
   * \param other   Eigen::Matrix [yaw; pitch; roll]
   */
  explicit EulerAnglesZyx(const Base & other)
    : Base(other) {
  }

  //! Creates from other rotation
  /*!
   * \param other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit EulerAnglesZyx(const RotationBase<OTHER_DERIVED, Usage> & other)
    : Base(internal::ConversionTraits<EulerAnglesZyx, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  EulerAnglesZyx & operator =(const RotationBase<OTHER_DERIVED, Usage> & other) {
    *this = internal::ConversionTraits<EulerAnglesZyx, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  /*! \returns the inverse of the rotation
   */
  EulerAnglesZyx inverted() const {
    return (EulerAnglesZyx)getInverseRpy<PrimType, PrimType>(*this);
  }

  /*! \inverts the rotation
   */
  EulerAnglesZyx & inverted() {
    *this = (EulerAnglesZyx)getInverseRpy<PrimType, PrimType>(*this);
    return *this;
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using EulerAnglesZyxBase<EulerAnglesZyx<PrimType, Usage>, Usage>::operator*; // otherwise ambiguous RotationBase and Eigen
  using EulerAnglesZyxBase<EulerAnglesZyx<PrimType, Usage>, Usage>::operator==; // otherwise ambiguous RotationBase and Eigen

  inline Scalar yaw() const {
    return toImplementation()(0);
  }

  inline Scalar pitch() const {
    return toImplementation()(1);
  }

  inline Scalar roll() const {
    return toImplementation()(2);
  }

  inline Scalar & yaw() {
    return toImplementation()(0);
  }

  inline Scalar & pitch() {
    return toImplementation()(1);
  }

  inline Scalar & roll() {
    return toImplementation()(2);
  }

  inline Scalar z() const {
    return toImplementation()(0);
  }

  inline Scalar y() const {
    return toImplementation()(1);
  }

  inline Scalar x() const {
    return toImplementation()(2);
  }

  inline Scalar & z() {
    return toImplementation()(0);
  }

  inline Scalar & y() {
    return toImplementation()(1);
  }

  inline Scalar & x() {
    return toImplementation()(2);
  }

  EulerAnglesZyx & setIdentity() {
	  this->Implementation::setZero();
	  return *this;
  }

  EulerAnglesZyx getUnique() const {  // wraps angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
    EulerAnglesZyx ypr(rm::common::Mod(yaw()  +M_PI,2*M_PI)-M_PI,
                       rm::common::Mod(pitch()+M_PI,2*M_PI)-M_PI,
                       rm::common::Mod(roll() +M_PI,2*M_PI)-M_PI); // wraps all angles into [-pi,pi)
    if(ypr.pitch() >= M_PI/2)
    {
      if(ypr.yaw() >= 0) {
        ypr.yaw() -= M_PI;
      } else {
        ypr.yaw() += M_PI;
      }

      ypr.pitch() = -(ypr.pitch()-M_PI);

      if(ypr.roll() >= 0) {
        ypr.roll() -= M_PI;
      } else {
        ypr.roll() += M_PI;
      }
    }
    else
      if(ypr.pitch() < -M_PI/2)
      {
        if(ypr.yaw() >= 0) {
          ypr.yaw() -= M_PI;
        } else {
          ypr.yaw() += M_PI;
        }

        ypr.pitch() = -(ypr.pitch()+M_PI);

        if(ypr.roll() >= 0) {
          ypr.roll() -= M_PI;
        } else {
          ypr.roll() += M_PI;
        }
      }
    return ypr;
  }

  friend std::ostream & operator << (std::ostream & out, const EulerAnglesZyx & ypr) {
    out << ypr.toImplementation().transpose();
    return out;
  }
};

typedef EulerAnglesZyx<double, RotationUsage::ACTIVE>  EulerAnglesZyxAD;
typedef EulerAnglesZyx<float,  RotationUsage::ACTIVE>  EulerAnglesZyxAF;
typedef EulerAnglesZyx<double, RotationUsage::PASSIVE> EulerAnglesZyxPD;
typedef EulerAnglesZyx<float,  RotationUsage::PASSIVE> EulerAnglesZyxPF;

template <typename PrimType, enum RotationUsage Usage>
using EulerAnglesYpr = EulerAnglesZyx<PrimType, Usage>;

typedef EulerAnglesYpr<double, RotationUsage::ACTIVE>  EulerAnglesYprAD;
typedef EulerAnglesYpr<float,  RotationUsage::ACTIVE>  EulerAnglesYprAF;
typedef EulerAnglesYpr<double, RotationUsage::PASSIVE> EulerAnglesYprPD;
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
  inline static eigen_implementation::AngleAxis<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & rpy) {
    return eigen_implementation::AngleAxis<DestPrimType, Usage>(eigen_implementation::getAngleAxisFromRpy<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::AngleAxis<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & ypr) {
    return eigen_implementation::AngleAxis<DestPrimType, Usage>(eigen_implementation::getAngleAxisFromYpr<SourcePrimType, DestPrimType>(ypr.toImplementation()));
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
  inline static eigen_implementation::RotationQuaternion<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & rpy) {
    return eigen_implementation::RotationQuaternion<DestPrimType, Usage>(eigen_implementation::getQuaternionFromRpy<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & ypr) {
    return eigen_implementation::RotationQuaternion<DestPrimType, Usage>(eigen_implementation::getQuaternionFromYpr<SourcePrimType, DestPrimType>(ypr.toImplementation()));
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
  inline static eigen_implementation::RotationMatrix<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & rpy) {
    return eigen_implementation::RotationMatrix<DestPrimType, Usage>(eigen_implementation::getRotationMatrixFromRpy<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::RotationMatrix<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & ypr) {
    return eigen_implementation::RotationMatrix<DestPrimType, Usage>(eigen_implementation::getRotationMatrixFromYpr<SourcePrimType, DestPrimType>(ypr.toImplementation()));
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
  inline static eigen_implementation::EulerAnglesXyz<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & rpy) {
    return eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>(rpy.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesXyz<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & ypr) {
    return eigen_implementation::EulerAnglesXyz<DestPrimType, Usage>(eigen_implementation::getRpyFromYpr<SourcePrimType, DestPrimType>(ypr.toImplementation()));
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
  inline static eigen_implementation::EulerAnglesZyx<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesXyz<SourcePrimType, Usage> & rpy) {
    return eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>(eigen_implementation::getYprFromRpy<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType, enum RotationUsage Usage>
class ConversionTraits<eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>, eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage>> {
 public:
  inline static eigen_implementation::EulerAnglesZyx<DestPrimType, Usage> convert(const eigen_implementation::EulerAnglesZyx<SourcePrimType, Usage> & ypr) {
    return eigen_implementation::EulerAnglesZyx<DestPrimType, Usage>(ypr.toImplementation().template cast<DestPrimType>());
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
  inline static typename get_matrix3X<eigen_implementation::EulerAnglesXyz<PrimType, Usage>>::template Matrix3X<Cols> rotate(const eigen_implementation::EulerAnglesXyz<PrimType, Usage> & rpy, const typename get_matrix3X<eigen_implementation::EulerAnglesXyz<PrimType, Usage>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType, Usage>(rpy).toImplementation() * m;
  }
};

template<typename PrimType, enum RotationUsage Usage>
class RotationTraits<eigen_implementation::EulerAnglesZyx<PrimType, Usage>> {
 public:
  template<typename get_matrix3X<eigen_implementation::EulerAnglesZyx<PrimType, Usage>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_implementation::EulerAnglesZyx<PrimType, Usage>>::template Matrix3X<Cols> rotate(const eigen_implementation::EulerAnglesZyx<PrimType, Usage> & ypr, const typename get_matrix3X<eigen_implementation::EulerAnglesZyx<PrimType, Usage>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType, Usage>(ypr).toImplementation() * m;
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

#endif /* ROTATIONEIGEN_HPP_ */
