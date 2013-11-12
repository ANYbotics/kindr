/*
 * QuaternionBase.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef QUATERNIONBASE_HPP_
#define QUATERNIONBASE_HPP_


namespace rm {
namespace quaternions {

namespace internal {

template<typename DEST, typename SOURCE>
class ConversionTraits {
  // DEST convert(const SOURCE & );
};

template<typename LEFT, typename RIGHT>
class MultiplicationTraits {
 public:
  inline static LEFT mult(const LEFT &l, const RIGHT &r){
    return LEFT(typename LEFT::Implementation(l.toImplementation() * r.toImplementation()));
  }
};

template<typename QUATERNION>
class ComparisonTraits {
// public:
//  inline static bool isequal(const QUATERNION & a, const QUATERNION & b) {
//    return a.toImplementation() == b.toImplemenentation();
//  }
};

} // namespace internal

template<typename DERIVED>
class QuaternionBase {
 public:
  DERIVED inverse() const;
  DERIVED conjugate() const;

  operator DERIVED & () {
    return static_cast<DERIVED &>(*this);
  }
  operator const DERIVED & () const {
    return static_cast<const DERIVED &>(*this);
  }

  const DERIVED & derived() const {
    return static_cast<const DERIVED &>(*this);
  }

//  template<typename OTHER_DERIVED>
//  DERIVED operator *(const QuaternionBase<OTHER_DERIVED> & other) const {
//    return internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult((DERIVED)*this, (DERIVED)other);
//  }

//  template<typename OTHER_DERIVED> // todo: ambiguous for eigen_implementation::UnitQuaternion (inheritance diamond)
//  bool operator ==(const QuaternionBase<OTHER_DERIVED> & other) const {
//    return internal::ComparisonTraits<DERIVED>::isequal((DERIVED)*this, (DERIVED)other); // cast to Quaternion
//  }

};



template<typename DERIVED>
class UnitQuaternionBase : public QuaternionBase<DERIVED> {
 public:
  typedef QuaternionBase<DERIVED> Base;

//  todo: how is it possible to use this?
//  DERIVED conjugate() const {
//    return DERIVED::conjugate();
//  }
//
//  DERIVED inverse() const {
//    return DERIVED::conjugate();
//  }

};




} // namespace quaternions
} // namespace rm

#endif /* QUATERNIONBASE_HPP_ */
