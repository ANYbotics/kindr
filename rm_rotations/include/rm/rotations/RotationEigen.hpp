/*
 * RotationEigen.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef ROTATIONEIGEN_HPP_
#define ROTATIONEIGEN_HPP_

#include "RotationBase.hpp"
#include "RotationEigenFunctions.hpp"

namespace rm {
namespace rotations {

namespace eigen_implementation {

template<typename PrimType>
class Quaternion : public quaternions::QuaternionBase<Quaternion<PrimType>>, private Eigen::Quaternion<PrimType> {
 public:
  typedef Eigen::Quaternion<PrimType> Base;
  typedef Eigen::Matrix<PrimType, 3, 1> ImagVector;

  // convert to and from Eigen::Quaternion
  explicit Quaternion(const Base & other)
      : Base(other) {
  }

//  AbstractImagQuatPart<3> imag() {
//
//  }

  Quaternion() = default;

  template<typename DERIVED>
  inline Quaternion(const Rotation<DERIVED> & other)
      : Eigen::Quaternion<PrimType>(internal::ConversionTraits<Quaternion, DERIVED>::convert(other)) {
  }

  template<typename OTHER_DERIVED>
  Quaternion & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }

  using Eigen::Quaternion<PrimType>::inverse;

  Quaternion inverse() {
    return Quaternion(Base::inverse());
  }

  Quaternion conjugate() {
    return Quaternion(Base::conjugate());
  }

//  Vector<3> imag();

  inline Eigen::Quaternion<PrimType> & toImplementation() {
    return static_cast<Eigen::Quaternion<PrimType> &>(*this);
  }
  inline const Eigen::Quaternion<PrimType> & toImplementation() const {
    return static_cast<const Eigen::Quaternion<PrimType> &>(*this);
  }
};

template<typename PrimType>
class UnitQuaternion : public UnitQuaternionBase<UnitQuaternion<PrimType>>, private Quaternion<PrimType> {
 private:
  typedef Quaternion<PrimType> Base;
 public:
  typedef Eigen::Matrix<PrimType, 3, 1> ImagVector;
  typedef typename internal::get_vector3d<UnitQuaternion>::type Vector3d;


  // convert to and from Quaternion
  explicit UnitQuaternion(const Base & other)
      : Base(other) {
  }

  // convert to and from Eigen::Quaternion
  explicit UnitQuaternion(const typename Base::Base & other)
      : Base(other) {
  }

//  AbstractImagQuatPart<3> imag() {
//
//  }


  UnitQuaternion() = default;

  using Base::toImplementation;

  template<typename DERIVED>
  inline UnitQuaternion(const Rotation<DERIVED> & other)
      : Eigen::Quaternion<PrimType>(internal::ConversionTraits<UnitQuaternion, DERIVED>::convert(other)) {
  }

  template<typename OTHER_DERIVED>
  UnitQuaternion & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }

  using Eigen::Quaternion<PrimType>::inverse;

  UnitQuaternion inverse() {
    return UnitQuaternion(Base::conjugate());
  }

//  Vector<3> imag();
};

template<typename PrimType>
UnitQuaternion<PrimType> operator *(const UnitQuaternion<PrimType> & a,
                                const UnitQuaternion<PrimType> & b) {

}

typedef UnitQuaternion<double> UnitQuaternionD;
typedef UnitQuaternion<float> UnitQuaternionF;


template<typename PrimType>
class AngleAxis : public AngleAxisBase<AngleAxis<PrimType>>, Eigen::AngleAxis<PrimType> {
public:
  typedef Eigen::Matrix<PrimType, 3, 1> Vector3d;

  template<typename DERIVED>
  inline explicit AngleAxis(const Rotation<DERIVED> & other)
      : Eigen::AngleAxis<PrimType>(
          internal::ConversionTraits<AngleAxis, DERIVED>::convert(
              static_cast<const DERIVED &>(other)
              )) {
  }

  template<typename OTHER_DERIVED>
  AngleAxis & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }

};

typedef AngleAxis<double> AngleAxisD;
typedef AngleAxis<float> AngleAxisF;


}  // namespace eigen_implementation

namespace internal {

template<typename PrimType>
class get_vector3d<eigen_implementation::UnitQuaternion<PrimType>>{
 public:
  typedef Eigen::Matrix<PrimType, 3, 1> type;
};

template<typename PrimType>
class get_vector3d<eigen_implementation::AngleAxis<PrimType>>{
 public:
  typedef Eigen::Matrix<PrimType, 3, 1> type;
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::UnitQuaternion<SourcePrimType>> {
public:
  inline static Eigen::AngleAxis<DestPrimType> convert(const eigen_implementation::UnitQuaternion<SourcePrimType> & q) {
    return getAngleAxisFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation());
  }
};


template<typename PrimType>
class RotationTraits<eigen_implementation::UnitQuaternion<PrimType>> {
 public:
   static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::UnitQuaternion<PrimType> & r, const Eigen::Matrix<PrimType, 3, 1> & v){
     return r.toImplementation() * v;
   }
};

}
}
}

#endif /* ROTATIONEIGEN_HPP_ */
