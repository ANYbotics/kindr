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

template<typename ROTATION>
class get_vector3 {
  // typedef VECTOR type;
};

template<typename ROTATION> // TODO: ok
class get_matrix3X {
  // typedef MATRIX type;
};

} // namespace internal

template<typename DERIVED>
class QuaternionBase {
 public:
  DERIVED inverse();
  DERIVED conjugate();

  operator DERIVED & () {
    return static_cast<DERIVED &>(*this);
  }
  operator const DERIVED & () const {
    return static_cast<const DERIVED &>(*this);
  }

  const DERIVED & derived() const {
    return static_cast<const DERIVED &>(*this);
  }

  //  typename Implementation::ImagVector imagVector();

  //  friend std::ostream & operator <<(std::ostream & out,
  //                                    const QuaternionBase & quat) {
  //    out << "(" << quat.real() << ", "
  //        << static_cast<Implementation&>(quat).imagVector() << ")";
  //    return out;
  //  }
};




template<typename DERIVED, typename OTHER_DERIVED> // TODO: ok?
QuaternionBase<DERIVED> operator *(const QuaternionBase<DERIVED> & a,
                                   const QuaternionBase<OTHER_DERIVED> & b) {
  return internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(a, b);
}

template<typename DERIVED, typename OTHER_DERIVED> // todo: ok?
bool operator ==(const QuaternionBase<DERIVED> & a, const QuaternionBase<OTHER_DERIVED> & b) {
  return internal::ComparisonTraits<DERIVED>::isequal(a.derived(), DERIVED(b));
}


template<typename DERIVED>
class UnitQuaternionBase : public QuaternionBase<DERIVED> {
 public:
  DERIVED inverse(){
    return DERIVED::conjugate();
  }
};

} // namespace quaternions
} // namespace rm

#endif /* QUATERNIONBASE_HPP_ */
