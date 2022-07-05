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

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/quaternions/QuaternionBase.hpp"

namespace kindr {


template<typename PrimType_>
class UnitQuaternion;

//! Implementation of a Quaternion based on Eigen::Quaternion
/*!
 * The Hamiltonian convention is used, where
 * Q = w + x*i + y*j + z*k and i*i=j*j=k*k=ijk=-1.
 *
 * The following two typedefs are provided for convenience:
 *   - QuaternionF for float
 *   - QuaternionD for double
 * \ingroup quaternions
 * \see rm::UnitQuaternion for an implementation of a unit quaternion
 * \see rm::rotations::RotationQuaternion for quaternions that represent a rotation
 */
template<typename PrimType_>
class Quaternion : public QuaternionBase<Quaternion<PrimType_>>, public Eigen::Quaternion<PrimType_> {
 private:
  typedef Eigen::Quaternion<PrimType_> Base;
 public:
  //! the implementation type, i.e., Eigen::Quaternion<>
  typedef Base Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType_ Scalar;
  //! the imaginary type, i.e., Eigen::Quaternion<>
  typedef Eigen::Matrix<PrimType_,3,1> Imaginary;
  //! quaternion as 4x1 matrix: [w; x; y; z]
  typedef Eigen::Matrix<PrimType_,4,1> Vector4;

  //! Default constructor creates a quaternion with all coefficients equal to zero
  Quaternion()
    : Base(Implementation(0,0,0,0)) {
  }

  /*! \brief Constructor using four scalars.
   *  \param w     first entry of the quaternion
   *  \param x     second entry of the quaternion
   *  \param y     third entry of the quaternion
   *  \param z     fourth entry of the quaternion
   */
  Quaternion(Scalar w, Scalar x, Scalar y, Scalar z)
    : Base(w,x,y,z) {
  }

  /*! \brief Constructor using real and imaginary part.
   *  \param real   real part (PrimType_)
   *  \param imag   imaginary part (Eigen::Matrix<PrimType_,3,1>)
   */
  Quaternion(Scalar real, const Imaginary& imag)
    : Base(real,imag(0),imag(1),imag(2)) {
  }

  /*! \brief Constructor using Eigen::Matrix<PrimType_,4,1>.
   *  \param other   Eigen::Matrix<PrimType_,4,1>
   */
  Quaternion(const Vector4& vector4)
    : Base(vector4(0),vector4(1),vector4(2),vector4(3)) {
  }

  // create from Eigen::Quaternion
  explicit Quaternion(const Base& other)
    : Base(other) {
  }

  /*! \returns the inverse of the quaternion
    */
  Quaternion inverted() const {
    return Quaternion(Implementation::inverse());
  }

  /*! \inverts the quaternion
    */
  Quaternion& invert() {
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
  Quaternion& conjugate() {
    *this = Quaternion(Implementation::conjugate());
    return *this;
  }

  Quaternion& operator =(const Quaternion<PrimType_>& other) = default;

  Quaternion& operator =(const UnitQuaternion<PrimType_>& other) {
    *this = Quaternion(other.toImplementation());
    return *this;
  }

//  bool operator ==(const Quaternion<PrimType_>& other) {
//	  return this->isEqual(other);
//  }

  template<typename PrimTypeIn_>
  Quaternion& operator ()(const Quaternion<PrimTypeIn_>& other) {
//	*this = other.template cast<PrimType_>();
	this->w() = static_cast<PrimType_>(other.w());
	this->x() = static_cast<PrimType_>(other.x());
	this->y() = static_cast<PrimType_>(other.y());
	this->z() = static_cast<PrimType_>(other.z());
	return *this;
  }

  template<typename PrimTypeIn_>
  Quaternion& operator ()(const UnitQuaternion<PrimTypeIn_>& other) {
//	*this = other.uq.template cast<PrimType_>(); // uq is private
	this->w() = static_cast<PrimType_>(other.w());
	this->x() = static_cast<PrimType_>(other.x());
	this->y() = static_cast<PrimType_>(other.y());
	this->z() = static_cast<PrimType_>(other.z());
	return *this;
  }

  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(*this);
  }
  inline const Implementation& toImplementation() const {
    return static_cast<const Implementation&>(*this);
  }

  using QuaternionBase<Quaternion<PrimType_>>::operator==;
  using QuaternionBase<Quaternion<PrimType_>>::operator*;

  inline Scalar w() const {
    return Base::w();
  }

  inline Scalar x() const {
    return Base::x();
  }

  inline Scalar y() const {
    return Base::y();
  }

  inline Scalar z() const {
    return Base::z();
  }

  inline Scalar& w() { // todo: attention: no assertion for unitquaternions!
    return Base::w();
  }

  inline Scalar& x() {
    return Base::x();
  }

  inline Scalar& y() {
    return Base::y();
  }

  inline Scalar& z() {
    return Base::z();
  }

  inline Scalar real() const {
    return Base::w();
  }

  inline Imaginary imaginary() const {
    return Imaginary(Base::x(),Base::y(),Base::z());
  }

  inline Vector4 vector() const {
    Vector4 vector4;
    vector4 << w(), x(), y(), z();
    return vector4;
  }

  inline Scalar norm() const {
    return Base::norm();
  }

  Quaternion normalized() const {
    return Quaternion(this->Base::normalized());
  }

  Quaternion& normalize() {
	  this->Base::normalize();
	  return *this;
  }

  Quaternion& setZero() {
    this->w() = Scalar(0.0);
    this->x() = Scalar(0.0);
    this->y() = Scalar(0.0);
    this->z() = Scalar(0.0);
    return *this;
  }

  /*! \brief Get zero element.
   *  \returns zero element
   */
  static Quaternion Zero() {
    return Quaternion();
  }

  UnitQuaternion<PrimType_> toUnitQuaternion() const {
    return UnitQuaternion<PrimType_>(this->Base::normalized());
  }

  /*! \brief Returns the quaternion matrix Qleft: q*p = Qleft(q)*p
   *  This function can be used to get the derivative of the concatenation with respect to the right quaternion.
   *  \returns the quaternion matrix Qleft
   */
  Eigen::Matrix<PrimType_,4,4> getQuaternionMatrix() {
    Eigen::Matrix<PrimType_,4,4> Qleft;
    Qleft(0,0) =  w();      Qleft(0,1) = -x();      Qleft(0,2) = -y();      Qleft(0,3) = -z();
    Qleft(1,0) =  x();      Qleft(1,1) =  w();      Qleft(1,2) = -z();      Qleft(1,3) =  y();
    Qleft(2,0) =  y();      Qleft(2,1) =  z();      Qleft(2,2) =  w();      Qleft(2,3) = -x();
    Qleft(3,0) =  z();      Qleft(3,1) = -y();      Qleft(3,2) =  x();      Qleft(3,3) =  w();
    return Qleft;
  }

  /*! \brief Returns the quaternion matrix Qright: q*p = Qright(p)*q
   *  This function can be used to get the derivative of the concatenation with respect to the left quaternion.
   *  \returns the quaternion matrix Qright
   */
  Eigen::Matrix<PrimType_,4,4> getConjugateQuaternionMatrix() {
    Eigen::Matrix<PrimType_,4,4> Qright;
    Qright(0,0) =  w();      Qright(0,1) = -x();      Qright(0,2) = -y();      Qright(0,3) = -z();
    Qright(1,0) =  x();      Qright(1,1) =  w();      Qright(1,2) =  z();      Qright(1,3) = -y();
    Qright(2,0) =  y();      Qright(2,1) = -z();      Qright(2,2) =  w();      Qright(2,3) =  x();
    Qright(3,0) =  z();      Qright(3,1) =  y();      Qright(3,2) = -x();      Qright(3,3) =  w();
    return Qright;
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
 * \see rm::Quaternion for an implementation of a generic quaternion
 * \see rm::rotations::RotationQuaternion for quaternions that represent a rotation
 */
template<typename PrimType_>
class UnitQuaternion : public UnitQuaternionBase<UnitQuaternion<PrimType_>> {
 private:
  Quaternion<PrimType_> unitQuternion_;
  typedef UnitQuaternionBase<UnitQuaternion<PrimType_>> Base;
 public:
  //! the implementation type, i.e., Eigen::Quaternion<>
  typedef typename Quaternion<PrimType_>::Implementation Implementation;
  //! the scalar type, i.e., the type of the coefficients
  typedef PrimType_ Scalar;
  //! the imaginary type, i.e., Eigen::Quaternion<>
  typedef Eigen::Matrix<PrimType_,3,1> Imaginary;
  //! quaternion as 4x1 matrix: [w; x; y; z]
  typedef Eigen::Matrix<PrimType_,4,1> Vector4;

  //! Default Constructor initializes the unit quaternion to identity
  UnitQuaternion()
    : unitQuternion_(Implementation::Identity()) {
  }

  //! Constructor to create unit quaternion from coefficients
  /*! Q = w + x*i + y*j + z*k
   * \param   w   scalar
   * \param   x   vector index 1
   * \param   y   vector index 2
   * \param   z   vector index 3
   */
  UnitQuaternion(Scalar w, Scalar x, Scalar y, Scalar z)
    : unitQuternion_(w,x,y,z) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1.0), static_cast<Scalar>(1e-2), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using real and imaginary part.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param real   real part (PrimType_)
   *  \param imag   imaginary part (Eigen::Matrix<PrimType_,3,1>)
   */
  UnitQuaternion(Scalar real, const Imaginary& imag)
    : unitQuternion_(real,imag) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1.0), static_cast<Scalar>(1e-2), "Input quaternion has not unit length.");
  }

  /*! \brief Constructor using Eigen::Matrix<PrimType_,4,1>.
   *  In debug mode, an assertion is thrown if the quaternion has not unit length.
   *  \param other   Eigen::Matrix<PrimType_,4,1>
   */
  UnitQuaternion(const Vector4& vector4)
    : unitQuternion_(vector4(0),vector4(1),vector4(2),vector4(3)) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1.0), static_cast<Scalar>(1e-2), "Input quaternion has not unit length.");
  }

  //! Constructor to create unit quaternion from Quaternion
  explicit UnitQuaternion(const Quaternion<PrimType_>& other)
    : unitQuternion_(other.toImplementation()) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1.0), static_cast<Scalar>(1e-2), "Input quaternion has not unit length.");
  }

  //! Constructor to create unit quaternion from Eigen::Quaternion
  /*!
   * \param other Eigen::Quaternion
   */
  explicit UnitQuaternion(const Implementation& other)
    : unitQuternion_(other) {
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1.0), static_cast<Scalar>(1e-2), "Input quaternion has not unit length.");
  }

  UnitQuaternion& operator =(const UnitQuaternion<PrimType_>& other) = default;

  template<typename PrimTypeIn_>
  UnitQuaternion& operator ()(const UnitQuaternion<PrimTypeIn_>& other) {
//	uq = other.uq;
	this->w() = static_cast<PrimType_>(other.w());
	this->x() = static_cast<PrimType_>(other.x());
	this->y() = static_cast<PrimType_>(other.y());
	this->z() = static_cast<PrimType_>(other.z());
	return *this;
  }

  template<typename PrimTypeIn_>
  UnitQuaternion& operator ()(const Quaternion<PrimTypeIn_>& other) {
//		*this = (UnitQuaternion)quat;
//	uq = other.template cast<PrimType_>();
	this->w() = static_cast<PrimType_>(other.w());
	this->x() = static_cast<PrimType_>(other.x());
	this->y() = static_cast<PrimType_>(other.y());
	this->z() = static_cast<PrimType_>(other.z());
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, norm(), static_cast<Scalar>(1.0), 1e-2, "Input quaternion has not unit length.");
	return *this;
  }

//  UnitQuaternion<PrimType_> operator *(const UnitQuaternion<PrimType_>& other) {
//	  return UnitQuaternion<PrimType_>(this->uq * other.uq);
//  }
//
//  Quaternion<PrimType_> operator *(const Quaternion<PrimType_>& other) {
//	  return Quaternion<PrimType_>(this->uq * other);
//  }
//  bool operator ==(const UnitQuaternion<PrimType_>& other) {
//	  return this->uq == other.uq;
//  }
//
//  bool operator ==(const Quaternion<PrimType_>& other) {
//	  return this->uq == other.uq;
//  }

  inline Scalar w() const {
    return unitQuternion_.w();
  }

  inline Scalar x() const {
    return unitQuternion_.x();
  }

  inline Scalar y() const {
    return unitQuternion_.y();
  }

  inline Scalar z() const {
    return unitQuternion_.z();
  }

  inline Scalar& w() { // todo: attention: no assertion for unitquaternions!
    return unitQuternion_.w();
  }

  inline Scalar& x() {
    return unitQuternion_.x();
  }

  inline Scalar& y() {
    return unitQuternion_.y();
  }

  inline Scalar& z() {
    return unitQuternion_.z();
  }

  inline Scalar real() const {
    return unitQuternion_.w();
  }

  inline Imaginary imaginary() const {
    return Imaginary(unitQuternion_.x(),unitQuternion_.y(),unitQuternion_.z());
  }

  inline Vector4 vector() const {
    Vector4 vector4;
    vector4 << w(), x(), y(), z();
    return vector4;
  }

//  using Base::operator*;

//  using UnitQuaternionBase<UnitQuaternion<PrimType_>>::conjugate;

  /*! \returns the conjugate of the quaternion
    */
  UnitQuaternion conjugated() const {
    return UnitQuaternion(unitQuternion_.conjugated());
  }

  /*! \conjugates the quaternion
    */
  UnitQuaternion& conjugate() {
    unitQuternion_.conjugate();
    return *this;
  }

//  using Base::inverted;
//  using Base::invert;

//  /*! \returns the inverse of the quaternion which is the conjugate for unit quaternions
//    */
//  UnitQuaternion inverse() const {
//    return UnitQuaternion(Base::conjugate());
//  }

  Scalar norm() const {
    return unitQuternion_.norm();
  }

  /*! \brief Sets unit quaternion to identity.
   *  \returns reference
   */
  UnitQuaternion& setIdentity() {
    this->w() = Scalar(0.0);
    this->x() = Scalar(0.0);
    this->y() = Scalar(0.0);
    this->z() = Scalar(0.0);
    return *this;
  }

  /*! \brief Get identity unit quaternion.
   *  \returns identity unit quaternion
   */
  static UnitQuaternion Identity() {
    return UnitQuaternion(Implementation::Identity());
  }

  const Implementation& toImplementation() const {
    return unitQuternion_.toImplementation();
  }

  Implementation& toImplementation() {
    return unitQuternion_.toImplementation();
  }

  /*! \brief Returns the quaternion matrix Qleft: q*p = Qleft(q)*p
   *  This function can be used to get the derivative of the concatenation with respect to the right quaternion.
   *  \returns the quaternion matrix Qleft
   */
  Eigen::Matrix<PrimType_,4,4> getQuaternionMatrix() {
    Eigen::Matrix<PrimType_,4,4> Qleft;
    Qleft(0,0) =  w();      Qleft(0,1) = -x();      Qleft(0,2) = -y();      Qleft(0,3) = -z();
    Qleft(1,0) =  x();      Qleft(1,1) =  w();      Qleft(1,2) = -z();      Qleft(1,3) =  y();
    Qleft(2,0) =  y();      Qleft(2,1) =  z();      Qleft(2,2) =  w();      Qleft(2,3) = -x();
    Qleft(3,0) =  z();      Qleft(3,1) = -y();      Qleft(3,2) =  x();      Qleft(3,3) =  w();
    return Qleft;
  }

  /*! \brief Returns the quaternion matrix Qright: q*p = Qright(p)*q
   *  This function can be used to get the derivative of the concatenation with respect to the left quaternion.
   *  \returns the quaternion matrix Qright
   */
  Eigen::Matrix<PrimType_,4,4> getConjugateQuaternionMatrix() {
    Eigen::Matrix<PrimType_,4,4> Qright;
    Qright(0,0) =  w();      Qright(0,1) = -x();      Qright(0,2) = -y();      Qright(0,3) = -z();
    Qright(1,0) =  x();      Qright(1,1) =  w();      Qright(1,2) =  z();      Qright(1,3) = -y();
    Qright(2,0) =  y();      Qright(2,1) = -z();      Qright(2,2) =  w();      Qright(2,3) =  x();
    Qright(3,0) =  z();      Qright(3,1) =  y();      Qright(3,2) = -x();      Qright(3,3) =  w();
    return Qright;
  }
};

//! Unit quaternion using double
typedef UnitQuaternion<double> UnitQuaternionD;
//! Unit quaternion using float
typedef UnitQuaternion<float> UnitQuaternionF;



} // namespace kindr

