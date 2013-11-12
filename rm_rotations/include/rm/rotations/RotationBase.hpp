/*
 * RotationBase.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef ROTATIONBASE_HPP_
#define ROTATIONBASE_HPP_

#include "rm/quaternions/QuaternionBase.hpp"

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

template<typename ROTATION>
class get_matrix3X {
  //  typedef int IndexType; // find type of Cols (e.g. Eigen::Dynamic)
  //  template <IndexType Cols> Matrix3X {
  //    typedef MATRIX type;
  //  }
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


//! Representation of a generic rotation
/*!
 * @ingroup rotations
 */
template<typename DERIVED>
class Rotation {
 public:

  /*! Inverses the rotation
   *
   * @return inverse of the rotation
   */
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

//  typename internal::get_vector3<DERIVED>::type rotate(const typename internal::get_vector3<DERIVED>::type & v) {
//    return internal::RotationTraits<DERIVED>::rotate(*this, v);
//  }

  template <typename internal::get_matrix3X<DERIVED>::IndexType Cols>
  typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> rotate(typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> & m) {
    return internal::RotationTraits<DERIVED>::rotate((DERIVED)*this, m);
  }

  template <typename internal::get_matrix3X<DERIVED>::IndexType Cols>
  typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> inverserotate(typename internal::get_matrix3X<DERIVED>::template Matrix3X<Cols> & m) {
    return internal::RotationTraits<DERIVED>::rotate(((DERIVED)*this).inverse(), m); // todo: may be optimized
  }

  template<typename OTHER_DERIVED>
  DERIVED operator *(const Rotation<OTHER_DERIVED> & other) const {
    return internal::MultiplicationTraits<DERIVED,DERIVED>::mult((DERIVED)*this, (DERIVED)other);
  }

  template<typename OTHER_DERIVED>
  bool operator ==(const Rotation<OTHER_DERIVED> & other) const {
    return internal::ComparisonTraits<DERIVED>::isequal((DERIVED)*this, (DERIVED)other);
  }
};




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
