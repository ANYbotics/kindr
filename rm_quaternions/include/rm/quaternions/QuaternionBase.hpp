/*
 * QuaternionBase.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef QUATERNIONBASE_HPP_
#define QUATERNIONBASE_HPP_

#include "rm/common/common.hpp"

namespace rm {
//! Generic quaternion interface
/*! \ingroup quaternions
 */
namespace quaternions {
//! Internal stuff (only for developers)
namespace internal {

template<typename DEST, typename SOURCE>
class ConversionTraits {
  // DEST convert(const SOURCE & );
};

template<typename LEFT, typename RIGHT>
class MultiplicationTraits {
 public:
  inline static typename LEFT::Implementation mult(const LEFT &l, const RIGHT &r){
    return typename LEFT::Implementation(l.toImplementation() * r.toImplementation());
  }
};

template<typename QUATERNION>
class ComparisonTraits {
 public:
  inline static bool isEqual(const QUATERNION & a, const QUATERNION & b){
    return (a.w() ==  b.w() &&
	          a.x() ==  b.x() &&
	          a.y() ==  b.y() &&
	          a.z() ==  b.z());
  }
};

} // namespace internal

//! Base class that defines the interface of a quaternion
/*! \ingroup quaternions
 * \see rm::rotations::RotationQuaternionBase for quaternions that represent a rotation
 */
template<typename DERIVED>
class QuaternionBase {
 public:
  /*! \inverts the quaternion
    */
  DERIVED & invert();

  /*! \returns the inverse of the quaternion
    */
  DERIVED inverted() const;

  /*! \conjugates the quaternion
    */
  DERIVED & conjugate();

  /*! \returns the conjugate of the quaternion
    */
  DERIVED conjugated() const;

  operator DERIVED & () {
    return static_cast<DERIVED &>(*this);
  }
  operator const DERIVED & () const {
    return static_cast<const DERIVED &>(*this);
  }

  DERIVED & derived() {
    return static_cast<DERIVED &>(*this);
  }
  const DERIVED & derived() const {
    return static_cast<const DERIVED &>(*this);
  }

  template<typename OTHER_DERIVED>
  DERIVED operator *(const QuaternionBase<OTHER_DERIVED> & other) const {
    return DERIVED(internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(this->derived(), other.derived()));
  }

//  template<typename OTHER_DERIVED>
//  DERIVED operator *(const QuaternionBase<OTHER_DERIVED> & other) const {
//    return internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(this->derived(), static_cast<DERIVED>(other));
//  }

  template<typename OTHER_DERIVED>
  bool operator ==(const QuaternionBase<OTHER_DERIVED> & other) const {
    return internal::ComparisonTraits<DERIVED>::isEqual(this->derived(), static_cast<DERIVED>(other)); // cast to Quaternion
  }

  friend std::ostream & operator << (std::ostream & out, const QuaternionBase<DERIVED> & quat) {
    out << quat.derived().w() << " " << quat.derived().x() << " " << quat.derived().y() << " " << quat.derived().z();
    return out;
  }
};


//! Base class that defines the interface of a unit quaternion
/*! \ingroup quaternions
 * \see rm::rotations::RotationQuaternionBase for quaternions that represent a rotation
 */
template<typename DERIVED>
class UnitQuaternionBase : public QuaternionBase<DERIVED> {
 public:
  typedef QuaternionBase<DERIVED> Base;

  DERIVED inverted() const {
    return Base::derived().conjugated();
  }

  DERIVED & invert() {
    return Base::derived().conjugate();
  }

  DERIVED conjugated() const;

  DERIVED & conjugate();

  template<typename OTHER_DERIVED>
  DERIVED operator *(const UnitQuaternionBase<OTHER_DERIVED> & other) const {
    return DERIVED(internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(this->derived(), other.derived()));
  }

  template<typename OTHER_DERIVED>
  OTHER_DERIVED operator *(const QuaternionBase<OTHER_DERIVED> & other) const {
    return OTHER_DERIVED(internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(this->derived(), other.derived()));
  }
};




} // namespace quaternions
} // namespace rm

#endif /* QUATERNIONBASE_HPP_ */
