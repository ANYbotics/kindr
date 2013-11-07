/*
 * QuaternionEigen.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef QUATERNIONEIGEN_HPP_
#define QUATERNIONEIGEN_HPP_

#include "QuaternionBase.hpp"

namespace rm {
namespace quaternions {
namespace eigen_implementation {

template<typename PrimType>
class Quaternion : public quaternions::QuaternionBase<Quaternion<PrimType>>, private Eigen::Quaternion<PrimType> {
  typedef Eigen::Quaternion<PrimType> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;
//  typedef Eigen::Matrix<PrimType, 3, 1> ImagVector;

//  AbstractImagQuatPart<3> imag() {
//
//  }

  Quaternion() = default;

  Quaternion(const PrimType & w, const PrimType & x, const PrimType & y, const PrimType & z)
    : Base(w,x,y,z) {
  }

  // create from Eigen::Quaternion
  explicit Quaternion(const Base & other)
      : Base(other) {
  }

//  // create from other rotation // TODO: necessary for non-unit-quaternion?
//  template<typename DERIVED>
//  inline Quaternion(const Rotation<DERIVED> & other)
//      : Base(internal::ConversionTraits<Quaternion, DERIVED>::convert(other)) {
////      : Base(internal::ConversionTraits<Quaternion, DERIVED>::convert(static_cast<const DERIVED &>(other))) { // TODO: use this?
//  }

    // TODO: necessary for non-unit-quaternion?
//  template<typename OTHER_DERIVED>
//  Quaternion & operator =(const Rotation<OTHER_DERIVED> & other) {
//
//    return *this;
//  }

//  using Base::inverse; // TODO: necessary?

  Quaternion inverse() {
    return Quaternion(Base::inverse());
  }

  Quaternion conjugate() {
    return Quaternion(Base::conjugate());
  }

//  Vector<3> imag();

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  friend std::ostream & operator << (std::ostream & out, const Quaternion & quat) {
    out << quat.toImplementation().w() << " " << quat.toImplementation().x() << " " << quat.toImplementation().y() << " " << quat.toImplementation().z();
    return out;
  }
};


typedef Quaternion<double> QuaternionD;
typedef Quaternion<float> QuaternionF;


template<typename PrimType>
class UnitQuaternion : public QuaternionBase<UnitQuaternion<PrimType>>, public Quaternion<PrimType> {
 private:
  typedef Quaternion<PrimType> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;
  typedef typename internal::get_vector3<UnitQuaternion>::type Vector3;
  typedef typename internal::get_matrix3X<UnitQuaternion>::type Matrix3X;

  UnitQuaternion() = default;

  UnitQuaternion(const PrimType & w, const PrimType & x, const PrimType & y, const PrimType & z)
    : Base(w,x,y,z) {
  }

  // create from Quaternion
  explicit UnitQuaternion(const Base & other)
      : Base(other) {
  }

  // create from Eigen::Quaternion
  explicit UnitQuaternion(const typename Implementation::Implementation & other)
      : Base(other) {
  }

  // create from other rotation
  template<typename DERIVED>
  inline UnitQuaternion(const QuaternionBase<DERIVED> & other)
      : Base(internal::ConversionTraits<UnitQuaternion, DERIVED>::convert(other)) {
  }

  template<typename OTHER_DERIVED>
  UnitQuaternion & operator =(const QuaternionBase<OTHER_DERIVED> & other) {
    return *this;
  }

  //  using Base::conjugate;

  UnitQuaternion conjugate() {
    return UnitQuaternion(Implementation::conjugate());
  }

  UnitQuaternion inverse() {
    return UnitQuaternion(Implementation::conjugate());
  }

  using Base::toImplementation;
};

typedef UnitQuaternion<double> UnitQuaternionD;
typedef UnitQuaternion<float> UnitQuaternionF;
}

namespace internal {

template<typename PrimType>
class get_vector3<eigen_implementation::UnitQuaternion<PrimType>>{
 public:
  typedef Eigen::Matrix<PrimType, 3, 1> type;
};

template<typename PrimType>
class get_matrix3X<eigen_implementation::UnitQuaternion<PrimType>>{
 public:
  typedef Eigen::Matrix<PrimType, 3, Eigen::Dynamic> type;
};



template<typename PrimType>
class ComparisonTraits<eigen_implementation::Quaternion<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::Quaternion<PrimType> & a, const eigen_implementation::Quaternion<PrimType> & b){
     return (a.toImplementation().w() ==  b.toImplementation().w() &&
             a.toImplementation().x() ==  b.toImplementation().x() &&
             a.toImplementation().y() ==  b.toImplementation().y() &&
             a.toImplementation().z() ==  b.toImplementation().z());
   }
};


} // namespace internal
} // namespace rotations
} // namespace rm


#endif /* QUATERNIONEIGEN_HPP_ */
