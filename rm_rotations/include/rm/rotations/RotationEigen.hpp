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

namespace eigen_implementation {


template<typename PrimType>
class AngleAxis : public AngleAxisBase<AngleAxis<PrimType>>, private Eigen::AngleAxis<PrimType> {
 private:
  typedef Eigen::AngleAxis<PrimType> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;
  typedef Eigen::Matrix<PrimType, 3, 1> Vector3;

  AngleAxis()
    : Base(Base::Identity()) {
  }

  AngleAxis(const Scalar & chi, const Scalar & v1, const Scalar & v2, const Scalar & v3)
    : Base(chi,Eigen::Matrix<Scalar,3,1>(v1,v2,v3)) {
  }

  // create from Eigen::AngleAxis
  explicit AngleAxis(const Base & other)
    : Base(other) {
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit AngleAxis(const Rotation<OTHER_DERIVED> & other)
    : Base(internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  AngleAxis & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  AngleAxis inverse() const {
    return AngleAxis(Base::inverse());
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using AngleAxisBase<AngleAxis<PrimType>>::operator*; // otherwise ambiguous RotationBase and Eigen

  inline Scalar angle() const {
    return Base::angle();
  }

  inline const Vector3 & axis() const {
    return Base::axis();
  }

  inline Scalar & angle() {
    return Base::angle();
  }

  inline Vector3 & axis() {
    return Base::axis();
  }

  friend std::ostream & operator << (std::ostream & out, const AngleAxis & a) {
    out << a.angle() << ", " << a.axis().transpose();
    return out;
  }
};

typedef AngleAxis<double> AngleAxisD;
typedef AngleAxis<float> AngleAxisF;



template<typename PrimType>
class RotationQuaternion : public RotationQuaternionBase<RotationQuaternion<PrimType>>, public quaternions::eigen_implementation::UnitQuaternion<PrimType> {
 private:
  typedef quaternions::eigen_implementation::UnitQuaternion<PrimType> Base;
 public:
  typedef typename Base::Implementation Implementation;
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

//  // create from Quaternion
//  explicit RotationQuaternion(const typename Base::Base & other)
//    : Base(other) {
//    ASSERT_SCALAR_NEAR(norm(), 1, 1e-6, "Input quaternion has not unit length.");
//  }

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

  using Base::norm;
  using Base::toImplementation;

  using RotationQuaternionBase<RotationQuaternion<PrimType>>::operator*; // otherwise ambiguous RotationBase and QuaternionBase
  using RotationQuaternionBase<RotationQuaternion<PrimType>>::operator==; // otherwise ambiguous RotationBase and Eigen
};

typedef RotationQuaternion<double> RotationQuaternionD;
typedef RotationQuaternion<float> RotationQuaternionF;


template<typename PrimType>
class RotationMatrix : public RotationMatrixBase<RotationMatrix<PrimType>>, private Eigen::Matrix<PrimType, 3, 3> {
 private:
  typedef Eigen::Matrix<PrimType, 3, 3> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;

  RotationMatrix()
    : Base(Base::Identity()) {
  }

  RotationMatrix(const Scalar & r11, const Scalar & r12, const Scalar & r13,
                 const Scalar & r21, const Scalar & r22, const Scalar & r23,
                 const Scalar & r31, const Scalar & r32, const Scalar & r33) {
    *this << r11,r12,r13,r21,r22,r23,r31,r32,r33;
    ASSERT_MATRIX_NEAR(*this * this->inverse(), Base::Identity(), 1e-6, "Input matrix is not orthogonal.");
  }

  // create from Eigen::Matrix
  explicit RotationMatrix(const Base & other)
      : Base(other) {
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

//  const RotationMatrix & setIdentity

  friend std::ostream & operator << (std::ostream & out, const RotationMatrix & R) {
    out << R.toImplementation();
    return out;
  }
};

typedef RotationMatrix<double> RotationMatrixD;
typedef RotationMatrix<float> RotationMatrixF;


template<typename PrimType>
class EulerAnglesRPY : public EulerAnglesRPYBase<EulerAnglesRPY<PrimType>>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;

  EulerAnglesRPY()
    : Base(Base::Zero()) {
  }

  EulerAnglesRPY(const Scalar & r, const Scalar & p, const Scalar & y)
    : Base(r,p,y) {
  }

  // create from Eigen::Matrix
  explicit EulerAnglesRPY(const Base & other)
    : Base(other) {
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit EulerAnglesRPY(const Rotation<OTHER_DERIVED> & other)
    : Base(internal::ConversionTraits<EulerAnglesRPY, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  EulerAnglesRPY & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<EulerAnglesRPY, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  EulerAnglesRPY inverse() const {
    return (EulerAnglesRPY)getInverseRPY<PrimType, PrimType>(*this);
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using EulerAnglesRPYBase<EulerAnglesRPY<PrimType>>::operator*; // otherwise ambiguous RotationBase and Eigen
  using EulerAnglesRPYBase<EulerAnglesRPY<PrimType>>::operator==; // otherwise ambiguous RotationBase and Eigen

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

  friend std::ostream & operator << (std::ostream & out, const EulerAnglesRPY & rpy) {
    out << rpy.toImplementation().transpose();
    return out;
  }
};

typedef EulerAnglesRPY<double> EulerAnglesRPYD;
typedef EulerAnglesRPY<float> EulerAnglesRPYF;


template<typename PrimType>
class EulerAnglesYPR : public EulerAnglesYPRBase<EulerAnglesYPR<PrimType>>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;

  EulerAnglesYPR()
    : Base(Base::Zero()) {
  }

  EulerAnglesYPR(const Scalar & y, const Scalar & p, const Scalar & r)
    : Base(y,p,r) {
  }

  // create from Eigen::Matrix
  explicit EulerAnglesYPR(const Base & other)
    : Base(other) {
  }

  // create from other rotation
  template<typename OTHER_DERIVED>
  inline explicit EulerAnglesYPR(const Rotation<OTHER_DERIVED> & other)
    : Base(internal::ConversionTraits<EulerAnglesYPR, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  EulerAnglesYPR & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<EulerAnglesYPR, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other));
    return *this;
  }

  EulerAnglesYPR inverse() const {
    return (EulerAnglesYPR)getInverseYPR<PrimType, PrimType>(*this);
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using EulerAnglesYPRBase<EulerAnglesYPR<PrimType>>::operator*; // otherwise ambiguous RotationBase and Eigen
  using EulerAnglesYPRBase<EulerAnglesYPR<PrimType>>::operator==; // otherwise ambiguous RotationBase and Eigen

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

  friend std::ostream & operator << (std::ostream & out, const EulerAnglesYPR & ypr) {
    out << ypr.toImplementation().transpose();
    return out;
  }
};

typedef EulerAnglesYPR<double> EulerAnglesYPRD;
typedef EulerAnglesYPR<float> EulerAnglesYPRF;



}  // namespace eigen_implementation


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
class get_matrix3X<eigen_implementation::EulerAnglesRPY<PrimType>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType, 3, Cols>;
};

template<typename PrimType>
class get_matrix3X<eigen_implementation::EulerAnglesYPR<PrimType>>{
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
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
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
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
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
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(getRPYFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(getRPYFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(getRPYFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(rpy.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(getRPYFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(getYPRFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(getYPRFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(getYPRFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(getYPRFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(ypr.toImplementation().template cast<DestPrimType>());
  }
};


template<typename PrimType>
class MultiplicationTraits<eigen_implementation::EulerAnglesRPY<PrimType>, eigen_implementation::EulerAnglesRPY<PrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<PrimType> mult(const eigen_implementation::EulerAnglesRPY<PrimType> & a, const eigen_implementation::EulerAnglesRPY<PrimType> & b) {
    return eigen_implementation::EulerAnglesRPY<PrimType>(eigen_implementation::RotationQuaternion<PrimType>(eigen_implementation::RotationQuaternion<PrimType>(a).toImplementation()*eigen_implementation::RotationQuaternion<PrimType>(b).toImplementation()));
  }
};

template<typename PrimType>
class MultiplicationTraits<eigen_implementation::EulerAnglesYPR<PrimType>, eigen_implementation::EulerAnglesYPR<PrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<PrimType> mult(const eigen_implementation::EulerAnglesYPR<PrimType> & a, const eigen_implementation::EulerAnglesYPR<PrimType> & b) {
    return eigen_implementation::EulerAnglesYPR<PrimType>(eigen_implementation::RotationQuaternion<PrimType>(eigen_implementation::RotationQuaternion<PrimType>(a).toImplementation()*eigen_implementation::RotationQuaternion<PrimType>(b).toImplementation()));
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
class RotationTraits<eigen_implementation::EulerAnglesRPY<PrimType>> {
 public:
  template<typename get_matrix3X<eigen_implementation::EulerAnglesRPY<PrimType>>::IndexType Cols>
   inline static typename get_matrix3X<eigen_implementation::EulerAnglesRPY<PrimType>>::template Matrix3X<Cols> rotate(const eigen_implementation::EulerAnglesRPY<PrimType> & rpy, const typename get_matrix3X<eigen_implementation::EulerAnglesRPY<PrimType>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType>(rpy).toImplementation() * m;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::EulerAnglesYPR<PrimType>> {
 public:
  template<typename get_matrix3X<eigen_implementation::EulerAnglesYPR<PrimType>>::IndexType Cols>
   inline static typename get_matrix3X<eigen_implementation::EulerAnglesYPR<PrimType>>::template Matrix3X<Cols> rotate(const eigen_implementation::EulerAnglesYPR<PrimType> & ypr, const typename get_matrix3X<eigen_implementation::EulerAnglesYPR<PrimType>>::template Matrix3X<Cols> & m){
    return eigen_implementation::RotationMatrix<PrimType>(ypr).toImplementation() * m;
   }
};



template<typename PrimType>
class ComparisonTraits<eigen_implementation::AngleAxis<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::AngleAxis<PrimType> & a, const eigen_implementation::AngleAxis<PrimType> & b){
     return (a.toImplementation().angle() ==  b.toImplementation().angle() && a.toImplementation().axis() ==  b.toImplementation().axis()) ||
            (a.toImplementation().angle() == -b.toImplementation().angle() && a.toImplementation().axis() == -b.toImplementation().axis());
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::RotationQuaternion<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::RotationQuaternion<PrimType> & a, const eigen_implementation::RotationQuaternion<PrimType> & b){
     return (a.toImplementation().w() ==  b.toImplementation().w() &&
             a.toImplementation().x() ==  b.toImplementation().x() &&
             a.toImplementation().y() ==  b.toImplementation().y() &&
             a.toImplementation().z() ==  b.toImplementation().z()) ||
            (a.toImplementation().w() == -b.toImplementation().w() &&
             a.toImplementation().x() == -b.toImplementation().x() &&
             a.toImplementation().y() == -b.toImplementation().y() &&
             a.toImplementation().z() == -b.toImplementation().z());
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::RotationMatrix<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::RotationMatrix<PrimType> & a, const eigen_implementation::RotationMatrix<PrimType> & b){
     return a.toImplementation() == b.toImplementation();
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::EulerAnglesRPY<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::EulerAnglesRPY<PrimType> & a, const eigen_implementation::EulerAnglesRPY<PrimType> & b){
     return a.toImplementation() == b.toImplementation();
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::EulerAnglesYPR<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::EulerAnglesYPR<PrimType> & a, const eigen_implementation::EulerAnglesYPR<PrimType> & b){
     return a.toImplementation() == b.toImplementation();
   }
};



} // namespace internal
} // namespace rotations
} // namespace rm

#endif /* ROTATIONEIGEN_HPP_ */
