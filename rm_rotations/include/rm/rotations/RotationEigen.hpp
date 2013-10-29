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
};

//template<typename PrimType>
//Quaternion<PrimType> operator *(const Quaternion<PrimType> & a,
//                                const Quaternion<PrimType> & b) {
//// TODO: insert something?
//}

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

  UnitQuaternion(const PrimType & w, const PrimType & x, const PrimType & y, const PrimType & z)
    : Base(w,x,y,z) {
  }

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
      : Base(internal::ConversionTraits<UnitQuaternion, DERIVED>::convert(other)) {
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

typedef RotationMatrix<double> RotationMatrixD;
typedef RotationMatrix<float> RotationMatrixF;


template<typename PrimType>
class EulerAnglesRPY : public EulerAnglesRPYBase<EulerAnglesRPY<PrimType>>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
//  typedef Eigen::Matrix<PrimType, 3, 1> Vector3;

  EulerAnglesRPY() = default;

  EulerAnglesRPY(const PrimType & r, const PrimType & p, const PrimType & y)
    : Base(r,p,y) {
  }

  // create from Eigen::Matrix
  explicit EulerAnglesRPY(const Base & other)
      : Base(other) {
  }

  // create from other rotation
  template<typename DERIVED>
  inline explicit EulerAnglesRPY(const Rotation<DERIVED> & other)
      : Base(internal::ConversionTraits<EulerAnglesRPY, DERIVED>::convert(static_cast<const DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  EulerAnglesRPY & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }

  EulerAnglesRPY inverse() {
    return getRPYFromQuaternion<PrimType, PrimType>(getInverseQuaternion<PrimType, PrimType>(getQuaternionFromRPY<PrimType, PrimType>(*this)));
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }
};

typedef EulerAnglesRPY<double> EulerAnglesRPYD;
typedef EulerAnglesRPY<float> EulerAnglesRPYF;


template<typename PrimType>
class EulerAnglesYPR : public EulerAnglesYPRBase<EulerAnglesYPR<PrimType>>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
//  typedef Eigen::Matrix<PrimType, 3, 1> Vector3;

  EulerAnglesYPR() = default;

  EulerAnglesYPR(const PrimType & y, const PrimType & p, const PrimType & r)
    : Base(y,p,r) {
  }

  // create from Eigen::Matrix
  explicit EulerAnglesYPR(const Base & other)
      : Base(other) {
  }

  // create from other rotation
  template<typename DERIVED>
  inline explicit EulerAnglesYPR(const Rotation<DERIVED> & other)
      : Base(internal::ConversionTraits<EulerAnglesYPR, DERIVED>::convert(static_cast<const DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  EulerAnglesYPR & operator =(const Rotation<OTHER_DERIVED> & other) {

    return *this;
  }

  EulerAnglesYPR inverse() {
    return getYPRFromQuaternion<PrimType, PrimType>(getInverseQuaternion<PrimType, PrimType>(getQuaternionFromYPR<PrimType, PrimType>(*this)));
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }
};

typedef EulerAnglesYPR<double> EulerAnglesYPRD;
typedef EulerAnglesYPR<float> EulerAnglesYPRF;





template<typename PrimType>
AngleAxis<PrimType> operator *(const AngleAxis<PrimType> & a,
                               const AngleAxis<PrimType> & b) {

}

template<typename PrimType>
AngleAxis<PrimType> operator *(const AngleAxis<PrimType> & a,
                               const UnitQuaternion<PrimType> & b) {

}

template<typename PrimType>
UnitQuaternion<PrimType> operator *(const UnitQuaternion<PrimType> & a,
                                    const UnitQuaternion<PrimType> & b) {
//  internal::MultiplicationTraits<UnitQuaternion<PrimType>, UnitQuaternion<PrimType>>::mult(a, b); // TODO: why no statements?
}

template<typename PrimType>
RotationMatrix<PrimType> operator *(const RotationMatrix<PrimType> & a,
                                    const RotationMatrix<PrimType> & b) {

}

template<typename PrimType>
EulerAnglesRPY<PrimType> operator *(const EulerAnglesRPY<PrimType> & a,
                                    const EulerAnglesRPY<PrimType> & b) {

}

template<typename PrimType>
EulerAnglesYPR<PrimType> operator *(const EulerAnglesYPR<PrimType> & a,
                                    const EulerAnglesYPR<PrimType> & b) {

}


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

template<typename PrimType>
class get_vector3<eigen_implementation::EulerAnglesRPY<PrimType>>{
 public:
  typedef Eigen::Matrix<PrimType, 3, 1> type;
};

template<typename PrimType>
class get_vector3<eigen_implementation::EulerAnglesYPR<PrimType>>{
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
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static Eigen::AngleAxis<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return getAngleAxisFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static Eigen::AngleAxis<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return getAngleAxisFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation());
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
class ConversionTraits<eigen_implementation::UnitQuaternion<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static Eigen::Quaternion<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return getQuaternionFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::UnitQuaternion<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static Eigen::Quaternion<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return getQuaternionFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation());
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

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 3> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return getRotationMatrixFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 3> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return getRotationMatrixFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation());
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 1> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return getRPYFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::UnitQuaternion<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 1> convert(const eigen_implementation::UnitQuaternion<SourcePrimType> & q) {
    return getRPYFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 1> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return getRPYFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 1> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return getRPYFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation());
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 1> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return getYPRFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::UnitQuaternion<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 1> convert(const eigen_implementation::UnitQuaternion<SourcePrimType> & q) {
    return getYPRFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 1> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return getYPRFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static Eigen::Matrix<DestPrimType, 3, 1> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return getYPRFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation());
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

template<typename LeftPrimType, typename RightPrimType> // TODO: correct MultiplicationTrait?
class MultiplicationTraits<eigen_implementation::EulerAnglesRPY<LeftPrimType>, eigen_implementation::EulerAnglesRPY<RightPrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<LeftPrimType> mult(const eigen_implementation::EulerAnglesRPY<LeftPrimType> & a, const eigen_implementation::EulerAnglesRPY<RightPrimType> & b) {
    return a.toImplementation()*b.toImplementation();
  }
};

template<typename LeftPrimType, typename RightPrimType> // TODO: correct MultiplicationTrait?
class MultiplicationTraits<eigen_implementation::EulerAnglesYPR<LeftPrimType>, eigen_implementation::EulerAnglesYPR<RightPrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<LeftPrimType> mult(const eigen_implementation::EulerAnglesYPR<LeftPrimType> & a, const eigen_implementation::EulerAnglesRPY<RightPrimType> & b) {
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

template<typename PrimType>
class RotationTraits<eigen_implementation::EulerAnglesRPY<PrimType>> {
 public:
   inline static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::EulerAnglesRPY<PrimType> & rpy, const Eigen::Matrix<PrimType, 3, 1> & v){
     return eigen_implementation::RotationMatrix<PrimType>(rpy).toImplementation() * v;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::EulerAnglesYPR<PrimType>> {
 public:
   inline static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::EulerAnglesYPR<PrimType> & ypr, const Eigen::Matrix<PrimType, 3, 1> & v){
     return eigen_implementation::RotationMatrix<PrimType>(ypr).toImplementation() * v;
   }
};

} // namespace internal
} // namespace rotations
} // namespace rm

#endif /* ROTATIONEIGEN_HPP_ */
