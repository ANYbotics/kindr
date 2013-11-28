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
#ifndef KINDER_QUATERNIONEIGEN_HPP_
#define KINDER_QUATERNIONEIGEN_HPP_

#include "kinder/common/common.hpp"
#include "QuaternionBase.hpp"
#include "kinder/common/assert_macros_eigen.hpp"
#include <Eigen/Geometry>

namespace kinder {
namespace quaternions {
//! Implementation based on the C++ Eigen library
namespace eigen_implementation {


//! Implementation of a Quaternion based on Eigen::Quaternion
/*!
 * The Hamiltonian convention is used, where
 * Q = w + x*i + y*j + z*k and i*i=j*j=k*k=ijk=-1.
 *
 * The following two typedefs are provided for convenience:
 *   - QuaternionF for float
 *   - QuaternionD for double
 * \ingroup quaternions
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
  //! the imaginary type, i.e., Eigen::Quaternion<>
  typedef Eigen::Matrix<PrimType,3,1> Imaginary;

  //! Default constructor creates a quaternion with all coefficients equal to zero
  Quaternion()
    : Base(Implementation(0,0,0,0)) {
  }

  Quaternion(const PrimType & w, const PrimType & x, const PrimType & y, const PrimType & z)
    : Base(w,x,y,z) {
  }

  Quaternion(const PrimType & w, const Imaginary & imag)
    : Base(w,imag(0),imag(1),imag(2)) {
  }

  // create from Eigen::Quaternion
  explicit Quaternion(const Base & other)
    : Base(other) {
  }

  /*! \returns the inverse of the quaternion
    */
  Quaternion inverted() const {
    return Quaternion(Implementation::inverse());
  }

  /*! \inverts the quaternion
    */
  Quaternion & invert() {
    *this = Quaternion(Implementation::inverse());
    return *this;
  }

  /*! \returns the conjugate of the quaternion
    */
  Quaternion conjugated() const {
    return Quaternion(Implementation::conjugate());
  }

  /*! \conjugates the quaternion
    */
  Quaternion & conjugate() {
    *this = Quaternion(Implementation::conjugate());
    return *this;
  }

  Quaternion & operator =(const Quaternion<PrimType> & other) {
    this->w() = other.w();
    this->x() = other.x();
    this->y() = other.y();
    this->z() = other.z();
    return *this;
  }

  Quaternion & operator =(const UnitQuaternion<PrimType> & other) {
    *this = Quaternion(other.toImplementation());
    return *this;
  }

//  bool operator ==(const Quaternion<PrimType> & other) {
//	  return this->isEqual(other);
//  }

  template<typename PrimTypeIn>
  Quaternion & operator ()(const Quaternion<PrimTypeIn> & other) {
//	*this = other.template cast<PrimType>();
	this->w() = static_cast<PrimType>(other.w());
	this->x() = static_cast<PrimType>(other.x());
	this->y() = static_cast<PrimType>(other.y());
	this->z() = static_cast<PrimType>(other.z());
	return *this;
  }

  template<typename PrimTypeIn>
  Quaternion & operator ()(const UnitQuaternion<PrimTypeIn> & other) {
//	*this = other.uq.template cast<PrimType>(); // uq is private
	this->w() = static_cast<PrimType>(other.w());
	this->x() = static_cast<PrimType>(other.x());
	this->y() = static_cast<PrimType>(other.y());
	this->z() = static_cast<PrimType>(other.z());
	return *this;
  }

  inline Implementation & toImplementation() {
    return static_cast<Implementation &>(*this);
  }
  inline const Implementation & toImplementation() const {
    return static_cast<const Implementation &>(*this);
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

  inline PrimType getReal() const {
    return Base::w();
  }

  inline Imaginary getImaginary() const {
    return Imaginary(Base::x(),Base::y(),Base::z());
  }

  inline PrimType norm() const {
    return Base::norm();
  }

  Quaternion normalized() const {
    return Quaternion(this->Base::normalized());
  }

  Quaternion & normalize() {
	  this->Base::normalize();
	  return *this;
  }

  UnitQuaternion<PrimType> toUnitQuaternion() const {
	return UnitQuaternion<PrimType>(this->Base::normalized());
  }
};

//! Quaternion using double
typedef Quaternion<double> QuaternionD;
//! Quaternion using float
typedef Quaternion<float> QuaternionF;

//! Implementation of a unit quaternion based on Eigen::Quaternion
/*! The Hamiltonian convention is used, where
 * Q = w + x*i + y*j + z*k and i*i=j*j=k*k=ijk=-1.
 *
 * The following two typedefs are provided for convenience:
 *   - UnitQuaternionF for float
 *   - UnitQuaternionD for double
 * \ingroup quaternions
 * \see rm::quaternions::eigen_implementation::Quaternion for an implementation of a generic quaternion
 * \see rm::rotations::eigen_implementation::RotationQuaternion for quaternions that represent a rotation
 */
template<typename PrimType>
class UnitQuaternion : public UnitQuaternionBase<UnitQuaternion<PrimType>> {
 private:
  Quaternion<PrimType> uq;
  typedef UnitQuaternionBase<UnitQuaternion<PrimType>> Base;
 public:
  //! the implementation type, i.e., Eigen::Quaternion<>
  typedef typename Quaternion<PrimType>::Implementation Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType Scalar;
  //! the imaginary type, i.e., Eigen::Quaternion<>
  typedef Eigen::Matrix<PrimType,3,1> Imaginary;


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
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), 1, 1e-6, "Input quaternion has not unit length.");
  }

  UnitQuaternion(const PrimType & w, const Imaginary & imag)
    : uq(w,imag) {
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), 1, 1e-6, "Input quaternion has not unit length.");
  }

  //! Constructor to create unit quaternion from Quaternion
  explicit UnitQuaternion(const Quaternion<PrimType> & other)
    : uq(other.toImplementation()) {
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), 1, 1e-6, "Input quaternion has not unit length.");
  }

  //! Constructor to create unit quaternion from Eigen::Quaternion
  /*!
   * \param other Eigen::Quaternion
   */
  explicit UnitQuaternion(const Implementation & other)
    : uq(other) {
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), 1, 1e-6, "Input quaternion has not unit length.");
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
  UnitQuaternion & operator ()(const UnitQuaternion<PrimTypeIn> & other) {
//	uq = other.uq;
	this->w() = static_cast<PrimType>(other.w());
	this->x() = static_cast<PrimType>(other.x());
	this->y() = static_cast<PrimType>(other.y());
	this->z() = static_cast<PrimType>(other.z());
	return *this;
  }

  template<typename PrimTypeIn>
  UnitQuaternion & operator ()(const Quaternion<PrimTypeIn> & other) {
//		*this = (UnitQuaternion)quat;
//	uq = other.template cast<PrimType>();
	this->w() = static_cast<PrimType>(other.w());
	this->x() = static_cast<PrimType>(other.x());
	this->y() = static_cast<PrimType>(other.y());
	this->z() = static_cast<PrimType>(other.z());
    KINDER_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), 1, 1e-6, "Input quaternion has not unit length.");
	return *this;
  }

//  UnitQuaternion<PrimType> operator *(const UnitQuaternion<PrimType> & other) {
//	  return UnitQuaternion<PrimType>(this->uq * other.uq);
//  }
//
//  Quaternion<PrimType> operator *(const Quaternion<PrimType> & other) {
//	  return Quaternion<PrimType>(this->uq * other);
//  }
//  bool operator ==(const UnitQuaternion<PrimType> & other) {
//	  return this->uq == other.uq;
//  }
//
//  bool operator ==(const Quaternion<PrimType> & other) {
//	  return this->uq == other.uq;
//  }

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

  inline PrimType getReal() const {
    return uq.w();
  }

  inline Imaginary getImaginary() const {
    return Imaginary(uq.x(),uq.y(),uq.z());
  }

//  using Base::operator*;

//  using UnitQuaternionBase<UnitQuaternion<PrimType>>::conjugate;

  /*! \returns the conjugate of the quaternion
    */
  UnitQuaternion conjugated() const {
    return UnitQuaternion(uq.conjugated());
  }

  /*! \conjugates the quaternion
    */
  UnitQuaternion & conjugate() {
    uq.conjugate();
    return *this;
  }

//  using Base::inverted;
//  using Base::invert;

//  /*! \returns the inverse of the quaternion which is the conjugate for unit quaternions
//    */
//  UnitQuaternion inverse() const {
//    return UnitQuaternion(Base::conjugate());
//  }

  PrimType norm() const {
	return uq.norm();
  }

  const Implementation & toImplementation() const {
	return uq.toImplementation();
  }

  Implementation & toImplementation() {
	return uq.toImplementation();
  }
};

//! Unit quaternion using double
typedef UnitQuaternion<double> UnitQuaternionD;
//! Unit quaternion using float
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


//template<typename PrimType>
//class MultiplicationTraits<eigen_implementation::Quaternion<PrimType>, eigen_implementation::Quaternion<PrimType>> {
//public:
//  inline static eigen_implementation::Quaternion<PrimType> mult(const eigen_implementation::Quaternion<PrimType> & a, const eigen_implementation::Quaternion<PrimType> & b) {
//    return eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(a).toImplementation()*eigen_implementation::Quaternion<PrimType>(b).toImplementation()));
//  }
//};
//
//template<typename PrimType>
//class MultiplicationTraits<eigen_implementation::Quaternion<PrimType>, eigen_implementation::UnitQuaternion<PrimType>> {
//public:
//  inline static eigen_implementation::Quaternion<PrimType> mult(const eigen_implementation::Quaternion<PrimType> & a, const eigen_implementation::UnitQuaternion<PrimType> & b) {
//    return eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(a).toImplementation()*eigen_implementation::Quaternion<PrimType>(b).toImplementation()));
//  }
//};
//
//template<typename PrimType>
//class MultiplicationTraits<eigen_implementation::UnitQuaternion<PrimType>, eigen_implementation::Quaternion<PrimType>> {
//public:
//  inline static eigen_implementation::Quaternion<PrimType> mult(const eigen_implementation::UnitQuaternion<PrimType> & a, const eigen_implementation::Quaternion<PrimType> & b) {
//    return eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(a).toImplementation()*eigen_implementation::Quaternion<PrimType>(b).toImplementation()));
//  }
//};
//
//template<typename PrimType>
//class MultiplicationTraits<eigen_implementation::UnitQuaternion<PrimType>, eigen_implementation::UnitQuaternion<PrimType>> {
//public:
//  inline static eigen_implementation::UnitQuaternion<PrimType> mult(const eigen_implementation::UnitQuaternion<PrimType> & a, const eigen_implementation::UnitQuaternion<PrimType> & b) {
//    return eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(eigen_implementation::Quaternion<PrimType>(a).toImplementation()*eigen_implementation::Quaternion<PrimType>(b).toImplementation()));
//  }
//};


//template<typename PrimType>
//class ComparisonTraits<eigen_implementation::Quaternion<PrimType>> {
// public:
//   inline static bool isEqual(const eigen_implementation::Quaternion<PrimType> & a, const eigen_implementation::Quaternion<PrimType> & b){
//     return (a.w() ==  b.w() &&
//             a.x() ==  b.x() &&
//             a.y() ==  b.y() &&
//             a.z() ==  b.z());
//   }
//};


} // namespace internal
} // namespace quaternions
} // namespace rm


#endif /* KINDER_QUATERNIONEIGEN_HPP_ */
