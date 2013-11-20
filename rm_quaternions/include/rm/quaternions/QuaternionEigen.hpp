/*
 * QuaternionEigen.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef QUATERNIONEIGEN_HPP_
#define QUATERNIONEIGEN_HPP_

#include "rm/common/Common.hpp"
#include "QuaternionBase.hpp"

#include <Eigen/Geometry>


// forward declarations

namespace rm {
namespace quaternions {
//! Implementation of quaternions based on the C++ Eigen library
namespace eigen_implementation {

template<typename PrimTypeIn>
class UnitQuaternion;

} // namespace eigen_implementation
} // namespace quaternions

namespace rotations {
namespace eigen_implementation {

template<typename PrimTypeIn>
class RotationQuaternion;

} // namespace eigen_implementation
} // namespace rotations
} // namespace rm




namespace rm {
namespace quaternions {
namespace eigen_implementation {


//! Implementation of a Quaternion based on Eigen::Quaternion
/*!
 * The Hamiltonian convention is used, where
 * Q = w + x*i + y*j + z*k and i*i=j*j=k*k=ijk=-1.
 *
 * The following two typedefs are provided for convenience:
 *   - QuaternionF for float
 *   - QuaternionD for double
 *
 * \see rm::quaternions::eigen_implementation::UnitQuaternion for an implementation of a unit quaternion
 * \see rm::rotations::eigen_implementation::RotationQuaternion for quaternions that represent a rotation
 */
template<typename PrimType>
class Quaternion : public QuaternionBase<Quaternion<PrimType>>, private Eigen::Quaternion<PrimType> {
 private:
  typedef Eigen::Quaternion<PrimType> Base;
 public:
  //! the implementation type, i.e., Eigen::Quaternion<>
  typedef Base Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;

  //! Default constructor creates a quaternion with all coefficients equal to zero
  Quaternion()
    : Base(Implementation(0,0,0,0)) {
  }

  Quaternion(const PrimType & w, const PrimType & x, const PrimType & y, const PrimType & z)
    : Base(w,x,y,z) {
  }

  // create from Eigen::Quaternion
  explicit Quaternion(const Base & other)
    : Base(other) {
  }

  /*! \returns the inverse of the quaternion
    */
  Quaternion inverse() const {
    return Quaternion(Base::inverse());
  }

  /*! \returns the conjugate of the quaternion
    */
  Quaternion conjugate() const {
    return Quaternion(Base::conjugate());
  }

  template<typename OTHER_DERIVED>
  Quaternion & operator =(const QuaternionBase<OTHER_DERIVED> & other) {
    *this = static_cast<Quaternion>(other);
    return *this;
  }

  template<typename PrimTypeIn>
  Quaternion & operator ()(const Quaternion<PrimTypeIn> & quat) {
	*this = quat.template cast<PrimType>();
//	this->w() = static_cast<PrimType>(quat.w());
//	this->x() = static_cast<PrimType>(quat.x());
//	this->y() = static_cast<PrimType>(quat.y());
//	this->z() = static_cast<PrimType>(quat.z());
	return *this;
  }

  template<typename PrimTypeIn>
  Quaternion & operator ()(const UnitQuaternion<PrimTypeIn> & quat) {
	*this = quat.uq.template cast<PrimType>();
//	this->w() = static_cast<PrimType>(quat.w());
//	this->x() = static_cast<PrimType>(quat.x());
//	this->y() = static_cast<PrimType>(quat.y());
//	this->z() = static_cast<PrimType>(quat.z());
	return *this;
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  using QuaternionBase<Quaternion<PrimType>>::operator==;
  using QuaternionBase<Quaternion<PrimType>>::operator*;

  inline PrimType w() const {
    return Base::w();
  }

  inline PrimType x() const {
    return Base::x();
  }

  inline PrimType y() const {
    return Base::y();
  }

  inline PrimType z() const {
    return Base::z();
  }

  inline PrimType & w() { // todo: attention: no assertion for unitquaternions!
    return Base::w();
  }

  inline PrimType & x() {
    return Base::x();
  }

  inline PrimType & y() {
    return Base::y();
  }

  inline PrimType & z() {
    return Base::z();
  }

  inline PrimType norm() {
    return Implementation::norm();
  }

  Quaternion & normalize() {
	  this->Implementation::normalize();
	  return *this;
  }

  Quaternion normalized() const {
	  return Quaternion(this->Implementation::normalized());
  }

  UnitQuaternion<PrimType> toUnitQuaternion() const {
	return UnitQuaternion<PrimType>(this->Implementation::normalized());
  }
};

typedef Quaternion<double> QuaternionD;
typedef Quaternion<float> QuaternionF;

//! Implementation of a unit quaternion based on Eigen::Quaternion
/*! The Hamiltonian convention is used, where
 * Q = w + x*i + y*j + z*k and i*i=j*j=k*k=ijk=-1.
 *
 * The following two typedefs are provided for convenience:
 *   - UnitQuaternionF for float
 *   - UnitQuaternionD for double
 *
 * \see rm::quaternions::eigen_implementation::Quaternion for an implementation of a generic quaternion
 * \see rm::rotations::eigen_implementation::RotationQuaternion for quaternions that represent a rotation
 */
template<typename PrimType>
class UnitQuaternion : public UnitQuaternionBase<UnitQuaternion<PrimType>> {
 private:
  Quaternion<PrimType> uq;
  typedef Quaternion<PrimType> Base;
 public:
  //! the implementation type, i.e., Eigen::Quaternion<>
  typedef typename Base::Implementation Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;


  //! Default Constructor initializes the unit quaternion to identity
  UnitQuaternion()
    : uq(Implementation::Identity()) {
  }

  //! Constructor to create unit quaternion from coefficients
  /*! Q = w + x*i + y*j + z*k
   * \param   w   scalar
   * \param   x   vector index 1
   * \param   y   vector index 2
   * \param   z   vector index 3
   */
  UnitQuaternion(const PrimType & w, const PrimType & x, const PrimType & y, const PrimType & z)
    : uq(w,x,y,z) {
    ASSERT_SCALAR_NEAR(norm(), 1, 1e-6, "Input quaternion has not unit length.");
  }

  //! Constructor to create unit quaternion from Quaternion
  explicit UnitQuaternion(const Base & other)
    : uq(other.toImplementation()) {
    ASSERT_SCALAR_NEAR(norm(), 1, 1e-6, "Input quaternion has not unit length.");
  }

  //! Constructor to create unit quaternion from Eigen::Quaternion
  /*!
   * \param other Eigen::Quaternion
   */
  explicit UnitQuaternion(const Implementation & other)
    : uq(other) {
    ASSERT_SCALAR_NEAR(norm(), 1, 1e-6, "Input quaternion has not unit length.");
  }

  template<typename PrimTypeIn>
  UnitQuaternion & operator =(const UnitQuaternion<PrimTypeIn> & other) {
	uq = other.template cast<PrimType>();
//    this->w() = static_cast<PrimType>(other.w());
//    this->x() = static_cast<PrimType>(other.x());
//    this->y() = static_cast<PrimType>(other.y());
//    this->z() = static_cast<PrimType>(other.z());
    return *this;
  }

  template<typename PrimTypeIn>
  UnitQuaternion & operator ()(const UnitQuaternion<PrimTypeIn> & quat) {
	uq = quat.template cast<PrimType>();
	return *this;
  }

  template<typename PrimTypeIn>
  UnitQuaternion & operator ()(const Quaternion<PrimTypeIn> & quat) {
//		*this = (UnitQuaternion)quat;
	uq = quat.template cast<PrimType>();
    ASSERT_SCALAR_NEAR(norm(), 1, 1e-6, "Input quaternion has not unit length.");
	return *this;
  }

  UnitQuaternion<PrimType> operator *(const UnitQuaternion<PrimType> & other) {
	  return UnitQuaternion<PrimType>(this->uq * other.uq);
  }

  Quaternion<PrimType> operator *(const Quaternion<PrimType> & other) {
	  return Quaternion<PrimType>(this->uq * other);
  }

  inline PrimType w() const {
    return uq.w();
  }

  inline PrimType x() const {
    return uq.x();
  }

  inline PrimType y() const {
    return uq.y();
  }

  inline PrimType z() const {
    return uq.z();
  }

  inline PrimType & w() { // todo: attention: no assertion for unitquaternions!
    return uq.w();
  }

  inline PrimType & x() {
    return uq.x();
  }

  inline PrimType & y() {
    return uq.y();
  }

  inline PrimType & z() {
    return uq.z();
  }

//  using Base::operator*;

//  using UnitQuaternionBase<UnitQuaternion<PrimType>>::conjugate;
//  using UnitQuaternionBase<UnitQuaternion<PrimType>>::inverse;

  /*! \returns the conjugate of the quaternion
    */
  UnitQuaternion conjugate() const {
    return UnitQuaternion(uq.conjugate());
  }

//  /*! \returns the inverse of the quaternion which is the conjugate for unit quaternions
//    */
//  UnitQuaternion inverse() const {
//    return UnitQuaternion(Base::conjugate());
//  }

  PrimType norm() const {
	return uq.norm();
  }

  const Implementation & toImplementation() {
	return uq;
  }
};

typedef UnitQuaternion<double> UnitQuaternionD;
typedef UnitQuaternion<float> UnitQuaternionF;



//template<typename PrimType>
//Quaternion<PrimType> operator *(const Quaternion<PrimType> & a, const Quaternion<PrimType> & b) {
//  return internal::MultiplicationTraits<Quaternion<PrimType>, Quaternion<PrimType>>::mult(a, b);
//}
//
//template<typename PrimType>
//bool operator ==(const Quaternion<PrimType> & a, const Quaternion<PrimType> & b) {
//  return internal::ComparisonTraits<Quaternion<PrimType>>::isequal((Quaternion<PrimType>)a, (Quaternion<PrimType>)b);
//}


} // namespace eigen_implementation

namespace internal {


template<typename PrimType>
class MultiplicationTraits<eigen_implementation::Quaternion<PrimType>, eigen_implementation::Quaternion<PrimType>> {
public:
  inline static eigen_implementation::Quaternion<PrimType> mult(const eigen_implementation::Quaternion<PrimType> & a, const eigen_implementation::Quaternion<PrimType> & b) {
    return eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(a).toImplementation()*eigen_implementation::Quaternion<PrimType>(b).toImplementation()));
  }
};


template<typename PrimType>
class ComparisonTraits<eigen_implementation::Quaternion<PrimType>> {
 public:
   inline static bool isEqual(const eigen_implementation::Quaternion<PrimType> & a, const eigen_implementation::Quaternion<PrimType> & b){
     return (a.w() ==  b.w() &&
             a.x() ==  b.x() &&
             a.y() ==  b.y() &&
             a.z() ==  b.z());
   }
};


} // namespace internal
} // namespace quaternions
} // namespace rm


#endif /* QUATERNIONEIGEN_HPP_ */
