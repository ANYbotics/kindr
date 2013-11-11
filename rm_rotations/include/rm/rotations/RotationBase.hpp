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
    std::cout << "HERE2: " << std::endl << l << std::endl << r << std::endl;
    std::cout << "HERE3: " << (l.toImplementation() * r.toImplementation()).matrix() << std::endl;
    std::cout << "HERE4: " << typename LEFT::Implementation(l.toImplementation() * r.toImplementation()).matrix() << std::endl;
    std::cout << "HERE5: " << LEFT(typename LEFT::Implementation(l.toImplementation() * r.toImplementation())) << std::endl;
    return LEFT(typename LEFT::Implementation(l.toImplementation() * r.toImplementation()));
  }
};


template<typename ROTATION>
class get_vector3 {
  // typedef VECTOR type;
};

template<typename ROTATION>
class get_matrix3X {
//  typedef int IndexType;// todo find type of COls (e.g. Eigen::Dynamic)
//  template <IndexType Cols> Matrix3X {
//    typedef E ...
//  }
  // typedef MATRIX type;
};

template<typename ROTATION>
class RotationTraits {
  // inline static typename internal::get_vector3<DERIVED>::type rotate(const ROTATION & r, const typename internal::get_vector3<DERIVED>::type & );
};

template<typename ROTATION>
class ComparisonTraits {
 public:
  inline static bool isequal(const ROTATION & a, const ROTATION & b) {
    return a.toImplementation() == b.toImplementation();
  }
};

} // namespace internal



//template <typename DERIVED>
//class Vector3;

template<typename DERIVED>
class Rotation {
 public:
  Rotation inverse();

  Rotation() = default;
  Rotation(const DERIVED &) = delete; // on purpose!!

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

  template <typename internal::get_matrix3X<DERIVED>::IndexType Cols>
  typename internal::get_matrix3X<DERIVED>::Matrix3X<Cols>::type rotate(typename internal::get_matrix3X<DERIVED>::Matrix3X<Cols>::type & m) {
    return internal::RotationTraits<DERIVED>::rotate(*this, m);
  }


  template<typename OTHER_DERIVED>
  DERIVED operator *(const Rotation<OTHER_DERIVED> & other) const {
//    std::cout << "HERE1: " << std::endl << *this << std::endl << other << std::endl;
    return internal::MultiplicationTraits<DERIVED,DERIVED>::mult((DERIVED)*this, (DERIVED)other);
  }


  template<typename OTHER_DERIVED> // todo: ok?
  bool operator ==(const Rotation<OTHER_DERIVED> & other) const {
  //  return a.derived().toImplementation() == Rotation<DERIVED>(b).derived().toImplementation();
    return internal::ComparisonTraits<DERIVED>::isequal((DERIVED)*this, (DERIVED)other);
  }
};

//template<typename DERIVED, typename OTHER_DERIVED>
//Rotation<DERIVED> operator *(const Rotation<DERIVED> & a,
//                             const Rotation<OTHER_DERIVED> & b) {
//  return internal::MultiplicationTraits<DERIVED, OTHER_DERIVED>::mult(a, b);
//}



//template<typename DERIVED, typename OTHER_DERIVED> // todo: ok?
//bool operator ==(const Rotation<DERIVED> & a, const Rotation<OTHER_DERIVED> & b) {
////  return a.derived().toImplementation() == Rotation<DERIVED>(b).derived().toImplementation();
//  return a.derived().toImplementation() == DERIVED(b).toImplementation();
//}

//template<typename DERIVED, typename OTHER_DERIVED> // todo: ok?
//bool operator ==(const Rotation<DERIVED> & a, const Rotation<OTHER_DERIVED> & b) {
////  return a.derived().toImplementation() == Rotation<DERIVED>(b).derived().toImplementation();
//  return internal::ComparisonTraits<DERIVED>::isequal(a.derived(), (DERIVED)b);
//}



template<typename Implementation>
class AngleAxisBase : public Rotation<Implementation> {

  template<typename OTHER_DERIVED>
  AngleAxisBase & operator =(const Rotation<OTHER_DERIVED> & other) {
    return *this;
  }
};

template<typename Implementation>
class RotationQuaternionBase : public Rotation<Implementation> {

  template<typename OTHER_DERIVED>
  RotationQuaternionBase & operator =(const Rotation<OTHER_DERIVED> & other) {
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
