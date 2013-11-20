/*
 * RotationEigen.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef ROTATIONEIGEN_HPP_
#define ROTATIONEIGEN_HPP_

#include "rm/common/Common.hpp"
#include "rm/quaternions/QuaternionEigen.hpp"
#include "RotationBase.hpp"
#include "RotationEigenFunctions.hpp"

namespace rm {
namespace rotations {
//! Implementation of rotations based on the C++ Eigen library
namespace eigen_implementation {

//!  Angle-axis rotation
template<typename PrimType>
class AngleAxis : public AngleAxisBase<AngleAxis<PrimType>>, private Eigen::AngleAxis<PrimType> {
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
    ASSERT_SCALAR_NEAR(this->axis().norm(), 1, 1e-6, "Input rotation axis has not unit length.");
  }

  //! Constructor initializes the angle and the vector from an Eigen::Vector3
  /*!
   * In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param chi   rotation angle
   * \param v     rotation vector with unit length
   */
  AngleAxis(const Scalar & chi, const Vector3 & v)
    : Base(chi,v) {
    ASSERT_SCALAR_NEAR(this->axis().norm(), 1, 1e-6, "Input rotation axis has not unit length.");
  }

  //! Constructor initializes from Eigen::AngleAxis
  /*! In debug mode, an assertion is thrown if the rotation vector has not unit length.
   * \param other   Eigen::AngleAxis
   */
  explicit AngleAxis(const Base & other)
    : Base(other) {
    ASSERT_SCALAR_NEAR(this->axis().norm(), 1, 1e-6, "Input rotation axis has not unit length.");
  }

  //! Constructor initializes from another rotation
  /*!
   * \param other   other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit AngleAxis(const Rotation<OTHER_DERIVED> & other)
    : Base(internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  AngleAxis & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  /*! \returns the inverse
   */
  AngleAxis inverse() const {
    return AngleAxis(Base::inverse());
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

  using AngleAxisBase<AngleAxis<PrimType>>::operator*; // otherwise ambiguous RotationBase and Eigen

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
	this->angle() = 0;
	this->axis() << 1,0,0;
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
typedef AngleAxis<double> AngleAxisD;
//! Angle-axis rotation with float
typedef AngleAxis<float> AngleAxisF;


//! Quaternion rotation
template<typename PrimType>
class RotationQuaternion : public RotationQuaternionBase<RotationQuaternion<PrimType>>, public quaternions::eigen_implementation::UnitQuaternion<PrimType> {
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
    ASSERT_SCALAR_NEAR(norm(), 1, 1e-6, "Input quaternion has not unit length.");
  }

//  wWithRealScalarGreaterOrEqualZero();
//  xWithRealScalarGreaterOrEqualZero();
//  yWithRealScalarGreaterOrEqualZero();

//  const quaternions::eigen_implementation::UnitQuaternion<PrimType> & getUnitQuaternion() const;
//  quaternions::eigen_implementation::UnitQuaternion<PrimType> getUnitQuaternionWithRealScalarGreaterOrEqualZero() const;

  // create from UnitQuaternion
  explicit RotationQuaternion(const Base & other)
    : Base(other) {
  }

  // create from Eigen::Quaternion
  explicit RotationQuaternion(const Implementation & other)
    : Base(other) {
    ASSERT_SCALAR_NEAR(norm(), 1, 1e-6, "Input quaternion has not unit length.");
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit RotationQuaternion(const Rotation<OTHER_DERIVED> & other)
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
  RotationQuaternion & operator =(const Rotation<OTHER_DERIVED> & other) {
    RotationQuaternion result = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    this->w() = result.w();
    this->x() = result.x();
    this->y() = result.y();
    this->z() = result.z();
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
    ASSERT_SCALAR_NEAR(norm(), 1, 1e-6, "Input quaternion has not unit length.");
	return *this;
  }

//  using Base::inverse(); // todo: will have Base as output

  RotationQuaternion inverse() const {
    return RotationQuaternion(Base::inverse());
  }

  RotationQuaternion conjugate() const {
    return RotationQuaternion(Base::conjugate());
  }

  RotationQuaternion & setIdentity() {
//	  this->Implementation::setIdentity(); // inaccessible
	this->w() = static_cast<Scalar>(1); // todo
	this->x() = 0;
	this->y() = 0;
	this->z() = 0;
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

  using Base::operator *;
  using Base::operator ==;
//  using RotationQuaternionBase<RotationQuaternion<PrimType>>::operator*; // otherwise ambiguous RotationBase and QuaternionBase
//  using RotationQuaternionBase<RotationQuaternion<PrimType>>::operator==; // otherwise ambiguous RotationBase and Eigen
};

typedef RotationQuaternion<double> RotationQuaternionD;
typedef RotationQuaternion<float> RotationQuaternionF;

//! Rotation matrix
template<typename PrimType>
class RotationMatrix : public RotationMatrixBase<RotationMatrix<PrimType>>, private Eigen::Matrix<PrimType, 3, 3> {
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
    ASSERT_MATRIX_NEAR(*this * this->transpose(), Base::Identity(), 1e-6, "Input matrix is not orthogonal.");
    ASSERT_SCALAR_NEAR(this->determinant(), 1, 1e-6, "Input matrix determinant is not 1.");
  }

  // create from Eigen::Matrix
  explicit RotationMatrix(const Base & other)
    : Base(other) {
	ASSERT_MATRIX_NEAR(other * other.transpose(), Base::Identity(), 1e-6, "Input matrix is not orthogonal.");
	ASSERT_SCALAR_NEAR(other.determinant(), 1, 1e-6, "Input matrix determinant is not 1.");
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit RotationMatrix(const Rotation<OTHER_DERIVED> & other)
    : Base(internal::ConversionTraits<RotationMatrix, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  RotationMatrix & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<RotationMatrix, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  RotationMatrix inverse() const {
    return RotationMatrix(toImplementation().transpose());
  }

  RotationMatrix transpose() const {
    return RotationMatrix(toImplementation().transpose());
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

  using RotationMatrixBase<RotationMatrix<PrimType>>::operator*; // otherwise ambiguous RotationBase and Eigen
  using RotationMatrixBase<RotationMatrix<PrimType>>::operator==; // otherwise ambiguous RotationBase and Eigen

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

typedef RotationMatrix<double> RotationMatrixD;
typedef RotationMatrix<float> RotationMatrixF;

//! Euler angles with X-Y''-Z''' convention
template<typename PrimType>
class EulerAnglesXYZ : public EulerAnglesXYZBase<EulerAnglesXYZ<PrimType>>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  //! the base type, i.e., Eigen::Matrix
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  //! the implementation type, i.e., Eigen::Matrix
  typedef Base Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;

  EulerAnglesXYZ()
    : Base(Base::Zero()) {
  }

  EulerAnglesXYZ(const Scalar & r, const Scalar & p, const Scalar & y)
    : Base(r,p,y) {
  }

  // create from Eigen::Matrix
  explicit EulerAnglesXYZ(const Base & other)
    : Base(other) {
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit EulerAnglesXYZ(const Rotation<OTHER_DERIVED> & other)
    : Base(internal::ConversionTraits<EulerAnglesXYZ, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  EulerAnglesXYZ & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<EulerAnglesXYZ, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  EulerAnglesXYZ inverse() const {
    return (EulerAnglesXYZ)getInverseRPY<PrimType, PrimType>(*this);
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using EulerAnglesXYZBase<EulerAnglesXYZ<PrimType>>::operator*; // otherwise ambiguous RotationBase and Eigen
  using EulerAnglesXYZBase<EulerAnglesXYZ<PrimType>>::operator==; // otherwise ambiguous RotationBase and Eigen

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

  EulerAnglesXYZ & setIdentity() {
	  this->Implementation::setZero();
	  return *this;
  }

  EulerAnglesXYZ getUnique() const {  // wraps angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
	EulerAnglesXYZ rpy(rm::common::Mod(roll() +M_PI,2*M_PI)-M_PI,
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

  friend std::ostream & operator << (std::ostream & out, const EulerAnglesXYZ & rpy) {
    out << rpy.toImplementation().transpose();
    return out;
  }
};

typedef EulerAnglesXYZ<double> EulerAnglesXYZD;
typedef EulerAnglesXYZ<float> EulerAnglesXYZF;

template <typename PrimType>
using EulerAnglesRPY = EulerAnglesXYZ<PrimType>;
typedef EulerAnglesRPY<double> EulerAnglesRPYD;
typedef EulerAnglesRPY<float> EulerAnglesRPYF;


//! Euler angles with Z-Y''-X''' convention
template<typename PrimType>
class EulerAnglesZYX : public EulerAnglesZYXBase<EulerAnglesZYX<PrimType>>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  //! the base type, i.e., Eigen::Matrix
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  //! the implementation type, i.e., Eigen::Matrix
  typedef Base Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;

  //! Default constructor initializes all angles with zero
  EulerAnglesZYX()
    : Base(Base::Zero()) {
  }

  //! Constructor initializes all three angles
  /*!
   * \param yaw     yaw angle
   * \param pitch   pitch angle
   * \param roll    roll angle
   */
  EulerAnglesZYX(const Scalar & yaw, const Scalar & pitch, const Scalar & roll)
    : Base(yaw,pitch,roll) {
  }

  //! Creates from Eigen::Matrix
  /*!
   * \param other   Eigen::Matrix [yaw; pitch; roll]
   */
  explicit EulerAnglesZYX(const Base & other)
    : Base(other) {
  }

  //! Creates from other rotation
  /*!
   * \param other rotation
   */
  template<typename OTHER_DERIVED>
  inline explicit EulerAnglesZYX(const Rotation<OTHER_DERIVED> & other)
    : Base(internal::ConversionTraits<EulerAnglesZYX, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  EulerAnglesZYX & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<EulerAnglesZYX, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  EulerAnglesZYX inverse() const {
    return (EulerAnglesZYX)getInverseYPR<PrimType, PrimType>(*this);
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using EulerAnglesZYXBase<EulerAnglesZYX<PrimType>>::operator*; // otherwise ambiguous RotationBase and Eigen
  using EulerAnglesZYXBase<EulerAnglesZYX<PrimType>>::operator==; // otherwise ambiguous RotationBase and Eigen

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

  EulerAnglesZYX & setIdentity() {
	  this->Implementation::setZero();
	  return *this;
  }

  EulerAnglesZYX getUnique() const {  // wraps angles into [-pi,pi),[-pi/2,pi/2),[-pi,pi)
	EulerAnglesZYX ypr(rm::common::Mod(yaw()  +M_PI,2*M_PI)-M_PI,
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

  friend std::ostream & operator << (std::ostream & out, const EulerAnglesZYX & ypr) {
    out << ypr.toImplementation().transpose();
    return out;
  }
};

typedef EulerAnglesZYX<double> EulerAnglesZYXD;
typedef EulerAnglesZYX<float> EulerAnglesZYXF;

template <typename PrimType>
using EulerAnglesYPR = EulerAnglesZYX<PrimType>;
typedef EulerAnglesYPR<double> EulerAnglesYPRD;
typedef EulerAnglesYPR<float> EulerAnglesYPRF;



} // namespace eigen_implementation


namespace internal {


template<typename PrimType>
class get_matrix3X<eigen_implementation::AngleAxis<PrimType>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType>
class get_matrix3X<eigen_implementation::RotationQuaternion<PrimType>>{
 public:
  typedef int IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType>
class get_matrix3X<eigen_implementation::RotationMatrix<PrimType>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType>
class get_matrix3X<eigen_implementation::EulerAnglesXYZ<PrimType>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType>
class get_matrix3X<eigen_implementation::EulerAnglesZYX<PrimType>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};



template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & a) {
    return eigen_implementation::AngleAxis<DestPrimType>(a.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::EulerAnglesXYZ<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::EulerAnglesXYZ<SourcePrimType> & rpy) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::EulerAnglesZYX<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::EulerAnglesZYX<SourcePrimType> & ypr) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(q.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::EulerAnglesXYZ<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::EulerAnglesXYZ<SourcePrimType> & rpy) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::EulerAnglesZYX<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::EulerAnglesZYX<SourcePrimType> & ypr) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::RotationMatrix<DestPrimType>(R.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::EulerAnglesXYZ<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::EulerAnglesXYZ<SourcePrimType> & rpy) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::EulerAnglesZYX<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::EulerAnglesZYX<SourcePrimType> & ypr) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesXYZ<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesXYZ<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::EulerAnglesXYZ<DestPrimType>(getRPYFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesXYZ<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesXYZ<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::EulerAnglesXYZ<DestPrimType>(getRPYFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesXYZ<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesXYZ<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::EulerAnglesXYZ<DestPrimType>(getRPYFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesXYZ<DestPrimType>, eigen_implementation::EulerAnglesXYZ<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesXYZ<DestPrimType> convert(const eigen_implementation::EulerAnglesXYZ<SourcePrimType> & rpy) {
    return eigen_implementation::EulerAnglesXYZ<DestPrimType>(rpy.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesXYZ<DestPrimType>, eigen_implementation::EulerAnglesZYX<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesXYZ<DestPrimType> convert(const eigen_implementation::EulerAnglesZYX<SourcePrimType> & ypr) {
    return eigen_implementation::EulerAnglesXYZ<DestPrimType>(getRPYFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesZYX<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesZYX<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::EulerAnglesZYX<DestPrimType>(getYPRFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesZYX<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesZYX<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::EulerAnglesZYX<DestPrimType>(getYPRFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesZYX<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesZYX<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::EulerAnglesZYX<DestPrimType>(getYPRFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesZYX<DestPrimType>, eigen_implementation::EulerAnglesXYZ<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesZYX<DestPrimType> convert(const eigen_implementation::EulerAnglesXYZ<SourcePrimType> & rpy) {
    return eigen_implementation::EulerAnglesZYX<DestPrimType>(getYPRFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesZYX<DestPrimType>, eigen_implementation::EulerAnglesZYX<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesZYX<DestPrimType> convert(const eigen_implementation::EulerAnglesZYX<SourcePrimType> & ypr) {
    return eigen_implementation::EulerAnglesZYX<DestPrimType>(ypr.toImplementation().template cast<DestPrimType>());
  }
};


// todo: specify more multiplication traits?

template<typename PrimType>
class MultiplicationTraits<eigen_implementation::RotationMatrix<PrimType>, eigen_implementation::RotationMatrix<PrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<PrimType> mult(const eigen_implementation::RotationMatrix<PrimType> & a, const eigen_implementation::RotationMatrix<PrimType> & b) {
    return eigen_implementation::RotationMatrix<PrimType>(a.toImplementation()*b.toImplementation());
  }
};



template<typename PrimType>
class RotationTraits<eigen_implementation::AngleAxis<PrimType>> {
 public:
  template<typename get_matrix3X<eigen_implementation::AngleAxis<PrimType>>::IndexType Cols>
   inline static typename get_matrix3X<eigen_implementation::AngleAxis<PrimType>>::template Matrix3X<Cols> rotate(const eigen_implementation::AngleAxis<PrimType> & aa, const typename get_matrix3X<eigen_implementation::AngleAxis<PrimType>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType>(aa).toImplementation() * m;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::RotationQuaternion<PrimType>> {
 public:
  template<typename get_matrix3X<eigen_implementation::RotationQuaternion<PrimType>>::IndexType Cols>
   inline static typename get_matrix3X<eigen_implementation::RotationQuaternion<PrimType>>::template Matrix3X<Cols> rotate(const eigen_implementation::RotationQuaternion<PrimType> & p, const typename get_matrix3X<eigen_implementation::RotationQuaternion<PrimType>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType>(p).toImplementation() * m;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::RotationMatrix<PrimType>> {
 public:
  template<typename get_matrix3X<eigen_implementation::RotationMatrix<PrimType>>::IndexType Cols>
   inline static typename get_matrix3X<eigen_implementation::RotationMatrix<PrimType>>::template Matrix3X<Cols> rotate(const eigen_implementation::RotationMatrix<PrimType> & R, const typename get_matrix3X<eigen_implementation::RotationMatrix<PrimType>>::template Matrix3X<Cols> & m){
    return R.toImplementation() * m;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::EulerAnglesXYZ<PrimType>> {
 public:
  template<typename get_matrix3X<eigen_implementation::EulerAnglesXYZ<PrimType>>::IndexType Cols>
   inline static typename get_matrix3X<eigen_implementation::EulerAnglesXYZ<PrimType>>::template Matrix3X<Cols> rotate(const eigen_implementation::EulerAnglesXYZ<PrimType> & rpy, const typename get_matrix3X<eigen_implementation::EulerAnglesXYZ<PrimType>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType>(rpy).toImplementation() * m;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::EulerAnglesZYX<PrimType>> {
 public:
  template<typename get_matrix3X<eigen_implementation::EulerAnglesZYX<PrimType>>::IndexType Cols>
   inline static typename get_matrix3X<eigen_implementation::EulerAnglesZYX<PrimType>>::template Matrix3X<Cols> rotate(const eigen_implementation::EulerAnglesZYX<PrimType> & ypr, const typename get_matrix3X<eigen_implementation::EulerAnglesZYX<PrimType>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType>(ypr).toImplementation() * m;
   }
};



template<typename PrimType>
class ComparisonTraits<eigen_implementation::AngleAxis<PrimType>> {
 public:
   inline static bool isEqual(const eigen_implementation::AngleAxis<PrimType> & a, const eigen_implementation::AngleAxis<PrimType> & b){
     return a.toImplementation().angle() ==  b.toImplementation().angle() &&
    		a.toImplementation().axis()  ==  b.toImplementation().axis();
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::RotationQuaternion<PrimType>> {
 public:
   inline static bool isEqual(const eigen_implementation::RotationQuaternion<PrimType> & a, const eigen_implementation::RotationQuaternion<PrimType> & b){
     return a.toImplementation().w() ==  b.toImplementation().w() &&
            a.toImplementation().x() ==  b.toImplementation().x() &&
            a.toImplementation().y() ==  b.toImplementation().y() &&
            a.toImplementation().z() ==  b.toImplementation().z();
   }

   inline static bool isNear(const eigen_implementation::RotationQuaternion<PrimType> & a, const eigen_implementation::RotationQuaternion<PrimType> & b, PrimType tol){
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
