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
class AngleAxis : public AngleAxisBase<AngleAxis<PrimType>>, private Eigen::AngleAxis<PrimType> {
 private:
  typedef Eigen::AngleAxis<PrimType> Base;
 public:
//  typedef Eigen::Matrix<PrimType, 3, 1> Vector3;

  AngleAxis() = default;

  // create from Eigen::AngleAxis
  explicit AngleAxis(const Base & other)
      : Base(other) {
  }

  // create from other rotation
  template<typename DERIVED>
  inline explicit AngleAxis(const Rotation<DERIVED> & other)
      : Base(internal::ConversionTraits<AngleAxis, DERIVED>::convert(static_cast<const DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  AngleAxis & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }

  AngleAxis inverse() {
    return AngleAxis(Base::inverse());
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }
};

template<typename PrimType>
AngleAxis<PrimType> operator *(const AngleAxis<PrimType> & a,
                               const AngleAxis<PrimType> & b) {
  return AngleAxis<PrimType>(a*b);
}

typedef AngleAxis<double> AngleAxisD;
typedef AngleAxis<float> AngleAxisF;


template<typename PrimType>
class Quaternion : public quaternions::QuaternionBase<Quaternion<PrimType>>, private Eigen::Quaternion<PrimType> {
 public:
  typedef Eigen::Quaternion<PrimType> Base; // TODO: why public?
//  typedef Eigen::Matrix<PrimType, 3, 1> ImagVector;

//  AbstractImagQuatPart<3> imag() {
//
//  }

  Quaternion() = default;

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
  inline const Eigen::Quaternion<PrimType> & toImplementation() const {
    return static_cast<const Eigen::Quaternion<PrimType> &>(*this);
  }
};

template<typename PrimType>
Quaternion<PrimType> operator *(const Quaternion<PrimType> & a,
                                const Quaternion<PrimType> & b) {

}

typedef Quaternion<double> QuaternionD;
typedef Quaternion<float> QuaternionF;


template<typename PrimType>
class UnitQuaternion : public UnitQuaternionBase<UnitQuaternion<PrimType>>, private Quaternion<PrimType> {
 private:
  typedef Quaternion<PrimType> Base;
 public:
//  typedef Eigen::Matrix<PrimType, 3, 1> ImagVector;
  typedef typename internal::get_vector3<UnitQuaternion>::type Vector3;

  UnitQuaternion() = default;

  // create from Quaternion
  explicit UnitQuaternion(const Base & other)
      : Base(other) {
  }

  // create from Eigen::Quaternion
  explicit UnitQuaternion(const typename Base::Base & other)
      : Base(other) {
  }

//  AbstractImagQuatPart<3> imag() {
//
//  }

  // create from other rotation
  template<typename DERIVED>
  inline UnitQuaternion(const Rotation<DERIVED> & other)
      : Eigen::Quaternion<PrimType>(internal::ConversionTraits<UnitQuaternion, DERIVED>::convert(other)) {
  }

  template<typename OTHER_DERIVED>
  UnitQuaternion & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }

//  using Base::inverse; // TODO: necessary?

  UnitQuaternion inverse() {
    return UnitQuaternion(Base::conjugate());
  }

//  Vector<3> imag();

  using Base::toImplementation;
};

template<typename PrimType>
UnitQuaternion<PrimType> operator *(const UnitQuaternion<PrimType> & a,
                                    const UnitQuaternion<PrimType> & b) {

}

typedef UnitQuaternion<double> UnitQuaternionD;
typedef UnitQuaternion<float> UnitQuaternionF;


template<typename PrimType>
class RotationMatrix : public RotationMatrixBase<RotationMatrix<PrimType>>, private Eigen::Matrix<PrimType, 3, 3> {
 private:
  typedef Eigen::Matrix<PrimType, 3, 3> Base;
 public:
//  typedef Eigen::Matrix<PrimType, 3, 1> Vector3;

  RotationMatrix() = default;

  // create from Eigen::Matrix
  explicit RotationMatrix(const Base & other)
      : Base(other) {
  }

  // create from other rotation
  template<typename DERIVED>
  inline explicit RotationMatrix(const Rotation<DERIVED> & other)
      : Base(internal::ConversionTraits<RotationMatrix, DERIVED>::convert(static_cast<const DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  RotationMatrix & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }

  RotationMatrix inverse() {
    return RotationMatrix(Base::transpose());
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }
};

template<typename PrimType>
RotationMatrix<PrimType> operator *(const RotationMatrix<PrimType> & a,
                                    const RotationMatrix<PrimType> & b) {

}

typedef RotationMatrix<double> RotationMatrixD;
typedef RotationMatrix<float> RotationMatrixF;


}  // namespace eigen_implementation


namespace internal {

template<typename PrimType>
class get_vector3<eigen_implementation::AngleAxis<PrimType>>{
 public:
  typedef Eigen::Matrix<PrimType, 3, 1> type;
};

template<typename PrimType>
class get_vector3<eigen_implementation::UnitQuaternion<PrimType>>{
 public:
  typedef Eigen::Matrix<PrimType, 3, 1> type;
};

template<typename PrimType>
class get_vector3<eigen_implementation::RotationMatrix<PrimType>>{
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

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static Eigen::AngleAxis<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return getAngleAxisFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::UnitQuaternion<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static Eigen::Quaternion<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) { // TODO: UnitQuaternion?
    return getQuaternionFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::UnitQuaternion<DestPrimType>, eigen_implementation::UnitQuaternion<SourcePrimType>> { // TODO: Quat to Quat necesary?
public:
  inline static Eigen::Quaternion<DestPrimType> convert(const eigen_implementation::UnitQuaternion<SourcePrimType> & q) {
    return q.toImplementation().template cast<DestPrimType>();
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::UnitQuaternion<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static Eigen::Quaternion<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) { // TODO: UnitQuaternion?
    return getQuaternionFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 3> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return getRotationMatrixFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::UnitQuaternion<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 3> convert(const eigen_implementation::UnitQuaternion<SourcePrimType> & q) {
    return getRotationMatrixFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation());
  }
};


template<typename LeftPrimType, typename RightPrimType> // TODO: correct MultiplicationTrait?
class MultiplicationTraits<eigen_implementation::AngleAxis<LeftPrimType>, eigen_implementation::AngleAxis<RightPrimType>> {
public:
  inline static eigen_implementation::AngleAxis<LeftPrimType> mult(const eigen_implementation::AngleAxis<LeftPrimType> & a, const eigen_implementation::AngleAxis<RightPrimType> & b) {
    return a.toImplementation()*b.toImplementation();
  }
};

template<typename LeftPrimType, typename RightPrimType> // TODO: correct MultiplicationTrait?
class MultiplicationTraits<eigen_implementation::Quaternion<LeftPrimType>, eigen_implementation::Quaternion<RightPrimType>> {
public:
  inline static eigen_implementation::Quaternion<LeftPrimType> mult(const eigen_implementation::Quaternion<LeftPrimType> & a, const eigen_implementation::Quaternion<RightPrimType> & b) {
    return a.toImplementation()*b.toImplementation();
  }
};

template<typename LeftPrimType, typename RightPrimType> // TODO: correct MultiplicationTrait?
class MultiplicationTraits<eigen_implementation::RotationMatrix<LeftPrimType>, eigen_implementation::RotationMatrix<RightPrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<LeftPrimType> mult(const eigen_implementation::RotationMatrix<LeftPrimType> & a, const eigen_implementation::RotationMatrix<RightPrimType> & b) {
    return a.toImplementation()*b.toImplementation();
  }
};


template<typename PrimType>
class RotationTraits<eigen_implementation::AngleAxis<PrimType>> {
 public:
   inline static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::AngleAxis<PrimType> & aa, const Eigen::Matrix<PrimType, 3, 1> & v){
     return aa.toImplementation() * v;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::UnitQuaternion<PrimType>> {
 public:
  inline static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::UnitQuaternion<PrimType> & r, const Eigen::Matrix<PrimType, 3, 1> & v){
     return r.toImplementation() * v;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::RotationMatrix<PrimType>> {
 public:
  inline static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::RotationMatrix<PrimType> & R, const Eigen::Matrix<PrimType, 3, 1> & v){
     return R.toImplementation() * v;
   }
};

} // namespace internal
} // namespace rotations
} // namespace rm

#endif /* ROTATIONEIGEN_HPP_ */
