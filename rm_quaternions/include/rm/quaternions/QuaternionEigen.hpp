/*
 * QuaternionEigen.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef QUATERNIONEIGEN_HPP_
#define QUATERNIONEIGEN_HPP_

#include "QuaternionBase.hpp"
#include <Eigen/Geometry>

namespace rm {
namespace quaternions {
namespace eigen_implementation {

template<typename PrimType>
class Quaternion : public QuaternionBase<Quaternion<PrimType>>, private Eigen::Quaternion<PrimType> {
 private:
  typedef Eigen::Quaternion<PrimType> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;

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

  Quaternion inverse() const {
    return Quaternion(Base::inverse());
  }

  Quaternion conjugate() const {
    return Quaternion(Base::conjugate());
  }

  template<typename OTHER_DERIVED>
  Quaternion & operator =(const QuaternionBase<OTHER_DERIVED> & other) {
    *this = (Quaternion)other;
    return *this;
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

//  using QuaternionBase<Quaternion<PrimType>>::operator==;
//  using QuaternionBase<Quaternion<PrimType>>::operator*;

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

  friend std::ostream & operator << (std::ostream & out, const Quaternion & quat) {
    out << quat.toImplementation().w() << " " << quat.toImplementation().x() << " " << quat.toImplementation().y() << " " << quat.toImplementation().z();
    return out;
  }
};

typedef Quaternion<double> QuaternionD;
typedef Quaternion<float> QuaternionF;


template<typename PrimType>
class UnitQuaternion : public UnitQuaternionBase<UnitQuaternion<PrimType>>, public Quaternion<PrimType> {
 private:
  typedef Quaternion<PrimType> Base;
 public:
  typedef typename Base::Implementation Implementation;
  typedef PrimType Scalar;

  UnitQuaternion()
    : Base(Implementation::Identity()) {
  }

  UnitQuaternion(const PrimType & w, const PrimType & x, const PrimType & y, const PrimType & z)
    : Base(w,x,y,z) {
//    assert(near(getNorm(), PrimType(1), 1E-9), "input has not unit length"); // todo
  }

  // create from Quaternion
  explicit UnitQuaternion(const Base & other)
    : Base(other) {
    //    assert(near(getNorm(), PrimType(1), 1E-9), "input has not unit length"); // todo
  }

  // create from Eigen::Quaternion
  explicit UnitQuaternion(const Implementation & other)
    : Base(other) {
  }

  // operator= is must only work with UnitQuaternions or derived classes
  UnitQuaternion & operator =(const UnitQuaternion & other) {
    //    assert(near(getNorm(), PrimType(1), 1E-9), "input has not unit length"); // todo
    *this = (UnitQuaternion)other;
    return *this;
  }

//  using UnitQuaternionBase<UnitQuaternion<PrimType>>::conjugate;
//  using UnitQuaternionBase<UnitQuaternion<PrimType>>::inverse;

  UnitQuaternion conjugate() const {
    return UnitQuaternion(Base::conjugate());
  }

  UnitQuaternion inverse() const {
    return UnitQuaternion(Base::conjugate());
  }

  using Base::toImplementation;
};

typedef UnitQuaternion<double> UnitQuaternionD;
typedef UnitQuaternion<float> UnitQuaternionF;



template<typename PrimType>
Quaternion<PrimType> operator *(const Quaternion<PrimType> & a, const Quaternion<PrimType> & b) {
  return internal::MultiplicationTraits<Quaternion<PrimType>, Quaternion<PrimType>>::mult(a, b);
}

template<typename PrimType>
bool operator ==(const Quaternion<PrimType> & a, const Quaternion<PrimType> & b) {
  return internal::ComparisonTraits<Quaternion<PrimType>>::isequal((Quaternion<PrimType>)a, (Quaternion<PrimType>)b);
}


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
   inline static bool isequal(const eigen_implementation::Quaternion<PrimType> & a, const eigen_implementation::Quaternion<PrimType> & b){
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
