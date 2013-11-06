/*
 * RotationBase.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef ROTATIONBASE_HPP_
#define ROTATIONBASE_HPP_

#include "QuaternionBase.hpp"

namespace rm {
namespace rotations {

namespace internal {

template<typename DEST, typename SOURCE>
class ConversionTraits {
  // inline static DEST convert(const SOURCE & );
};

template<typename LEFT, typename RIGHT>
class MultiplicationTraits {
 public:
  inline static LEFT mult(const LEFT &l, const RIGHT &r){
    return LEFT(typename LEFT::Implementation(l.toImplementation() * r.toImplementation()));
  }
};


template<typename ROTATION>
class get_vector3 {
  // typedef VECTOR type;
};

//template<typename ROTATION> // todo: ok
//class get_matrix3X {
//  // typedef MATRIX type;
//};

template<typename ROTATION>
class RotationTraits {
  // inline static typename internal::get_vector3<DERIVED>::type rotate(const ROTATION & r, const typename internal::get_vector3<DERIVED>::type & );
};

} // namespace internal



//template <typename DERIVED>
//class Vector3;

template<typename DERIVED>
class Rotation {
 public:
  Rotation inverse();

  operator DERIVED & () {
    return static_cast<DERIVED &>(*this);
  }
  operator const DERIVED & () const {
    return static_cast<const DERIVED &>(*this);
  }

  const DERIVED & derived() const {
    return static_cast<const DERIVED &>(*this);
  }


  typename internal::get_vector3<DERIVED>::type rotate(typename internal::get_vector3<DERIVED>::type & v) {
    return internal::RotationTraits<DERIVED>::rotate(*this, v);
  }

//  typename internal::get_matrix3X<DERIVED>::type rotate(typename internal::get_matrix3X<DERIVED>::type & m) {
//    return internal::RotationTraits<DERIVED>::rotate(*this, m);
//  }
};

template<typename DERIVED, typename OTHER_DERIVED>
Rotation<DERIVED> operator *(const Rotation<DERIVED> & a,
                             const Rotation<OTHER_DERIVED> & b) {
  return internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(a, b);
}



template<typename DERIVED, typename OTHER_DERIVED> // todo: ok?
bool operator ==(const Rotation<DERIVED> & a, const Rotation<OTHER_DERIVED> & b) {
  return a.derived().toImplementation() == Rotation<DERIVED>(b).derived().toImplementation();
}


template<typename Implementation>
class RotationQuaternionBase : public Rotation<Implementation> {

//  typename Implementation::ImagVector imagVector();

//  friend std::ostream & operator <<(std::ostream & out,
//                                    const QuaternionBase & quat) {
//    out << "(" << quat.real() << ", "
//        << static_cast<Implementation&>(quat).imagVector() << ")";
//    return out;
//  }

};

//template<typename DERIVED>
//void foo(const UnitQuaternionBase<DERIVED> & q) {
//
//}


template<typename Implementation>
class AngleAxisBase : public Rotation<Implementation> {

  template<typename OTHER_DERIVED>
  AngleAxisBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }
};


template<typename Implementation>
class RotationMatrixBase : public Rotation<Implementation> {

  template<typename OTHER_DERIVED>
  RotationMatrixBase & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }
};


template<typename Implementation>
class EulerAnglesBase : public Rotation<Implementation> {

};


template<typename Implementation>
class EulerAnglesRPYBase : public EulerAnglesBase<Implementation> {

  template<typename OTHER_DERIVED>
  EulerAnglesRPYBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }
};


template<typename Implementation>
class EulerAnglesYPRBase : public EulerAnglesBase<Implementation> {

  template<typename OTHER_DERIVED>
  EulerAnglesYPRBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }
};


} // namespace rotations
} // namespace rm

#endif /* ROTATIONBASE_HPP_ */
