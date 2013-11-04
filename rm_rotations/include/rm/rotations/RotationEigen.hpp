/*
 * RotationEigen.hpp
 *
 *  Created on: Oct 28, 2013
 *      Author: remod
 */

#ifndef ROTATIONEIGEN_HPP_
#define ROTATIONEIGEN_HPP_

#include "QuaternionEigen.hpp"
#include "RotationBase.hpp"
#include "RotationEigenFunctions.hpp"

namespace rm {
namespace rotations {

namespace eigen_implementation {

template<typename PrimType>
class AngleAxis : public AngleAxisBase<AngleAxis<PrimType>>, private Eigen::AngleAxis<PrimType> {
  typedef Eigen::AngleAxis<PrimType> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;

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
class RotationQuaternion : public RotationQuaternionBase<RotationQuaternion<PrimType>>, public quaternions::eigen_implementation::UnitQuaternion<PrimType> {
 private:
  typedef quaternions::eigen_implementation::UnitQuaternion<PrimType> Base;
 public:
  typedef typename Base::Implementation Implementation;
  typedef PrimType Scalar;

//  typedef typename internal::get_vector3<RotationQuaternion>::type Vector3;

  RotationQuaternion() = default;

  RotationQuaternion(const PrimType & w, const PrimType & x, const PrimType & y, const PrimType & z)
    : Base(w,x,y,z) {
  }

  // create from Quaternion
  explicit RotationQuaternion(const Base & other)
      : Base(other) {
  }

  // create from Eigen::Quaternion
  explicit RotationQuaternion(const Implementation & other)
      : Base(other) {
  }

  // create from other rotation
  template<typename DERIVED>
  inline RotationQuaternion(const Rotation<DERIVED> & other)
      : Base(internal::ConversionTraits<RotationQuaternion, DERIVED>::convert(other)) {
  }

  template<typename OTHER_DERIVED>
  RotationQuaternion & operator =(const Rotation<OTHER_DERIVED> & other) {
    Base(internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other)); // todo: ok?
    return *this;
  }


  bool operator == (const RotationQuaternion & other) {
    return toImplementation() == other.toImplementation() || toImplementation() == -other.toImplementation();
  }

//  using Base::inverse; // TODO: necessary?

  RotationQuaternion inverse() {
    return RotationQuaternion(Base::conjugate());
  }

//  Vector<3> imag();

  using Base::toImplementation;
};

typedef RotationQuaternion<double> RotationQuaternionD;
typedef RotationQuaternion<float> RotationQuaternionF;


template<typename PrimType>
class RotationMatrix : public RotationMatrixBase<RotationMatrix<PrimType>>, private Eigen::Matrix<PrimType, 3, 3> {
 private:
  typedef Eigen::Matrix<PrimType, 3, 3> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;

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
    return RotationMatrix(toImplementation().transpose());
  }

  inline Implementation & toImplementation() {
    return static_cast<Implementation &>(*this);
  }
  inline const Implementation & toImplementation() const {
    return static_cast<const Implementation &>(*this);
  }
};

typedef RotationMatrix<double> RotationMatrixD;
typedef RotationMatrix<float> RotationMatrixF;


template<typename PrimType>
class EulerAnglesRPY : public EulerAnglesRPYBase<EulerAnglesRPY<PrimType>>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  typedef Base Implementation;
  typedef PrimType Scalar;

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
  typedef Base Implementation;
  typedef PrimType Scalar;

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
  return internal::MultiplicationTraits<AngleAxis<PrimType>, AngleAxis<PrimType>>::mult(a, b);
}

template<typename PrimType>
AngleAxis<PrimType> operator *(const AngleAxis<PrimType> & a,
                               const RotationQuaternion<PrimType> & b) {
  return internal::MultiplicationTraits<AngleAxis<PrimType>, RotationQuaternion<PrimType>>::mult(a, b);
}

template<typename PrimType> // todo: ok?
RotationQuaternion<PrimType> operator *(const RotationQuaternion<PrimType> & a,
                                        const RotationQuaternion<PrimType> & b) {
  return RotationQuaternion<PrimType> (quaternions::internal::MultiplicationTraits<quaternions::eigen_implementation::Quaternion<PrimType>, quaternions::eigen_implementation::Quaternion<PrimType>>::mult(a, b));
}

template<typename PrimType>
RotationMatrix<PrimType> operator *(const RotationMatrix<PrimType> & a,
                                    const RotationMatrix<PrimType> & b) {
  return internal::MultiplicationTraits<RotationMatrix<PrimType>, RotationMatrix<PrimType>>::mult(a, b);
}

template<typename PrimType>
EulerAnglesRPY<PrimType> operator *(const EulerAnglesRPY<PrimType> & a,
                                    const EulerAnglesRPY<PrimType> & b) {
  return internal::MultiplicationTraits<EulerAnglesRPY<PrimType>, EulerAnglesRPY<PrimType>>::mult(a, b);
}

template<typename PrimType>
EulerAnglesYPR<PrimType> operator *(const EulerAnglesYPR<PrimType> & a,
                                    const EulerAnglesYPR<PrimType> & b) {
  return internal::MultiplicationTraits<EulerAnglesYPR<PrimType>, EulerAnglesYPR<PrimType>>::mult(a, b);
}


}  // namespace eigen_implementation


namespace internal {

template<typename PrimType>
class get_vector3<eigen_implementation::AngleAxis<PrimType>>{
 public:
  typedef Eigen::Matrix<PrimType, 3, 1> type;
};

template<typename PrimType>
class get_vector3<eigen_implementation::RotationQuaternion<PrimType>>{
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
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & a) {
    return eigen_implementation::AngleAxis<DestPrimType>(a.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::AngleAxis<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::AngleAxis<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return eigen_implementation::AngleAxis<DestPrimType>(getAngleAxisFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(q.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationQuaternion<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationQuaternion<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return eigen_implementation::RotationQuaternion<DestPrimType>(getQuaternionFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::RotationMatrix<DestPrimType>(R.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::RotationMatrix<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::RotationMatrix<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return eigen_implementation::RotationMatrix<DestPrimType>(getRotationMatrixFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(getRPYFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(getRPYFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(getRPYFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(rpy.toImplementation().template cast<DestPrimType>());
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesRPY<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return eigen_implementation::EulerAnglesRPY<DestPrimType>(getRPYFromYPR<SourcePrimType, DestPrimType>(ypr.toImplementation()));
  }
};


template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::AngleAxis<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::AngleAxis<SourcePrimType> & aa) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(getYPRFromAngleAxis<SourcePrimType, DestPrimType>(aa.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::RotationQuaternion<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::RotationQuaternion<SourcePrimType> & q) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(getYPRFromQuaternion<SourcePrimType, DestPrimType>(q.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::RotationMatrix<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::RotationMatrix<SourcePrimType> & R) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(getYPRFromRotationMatrix<SourcePrimType, DestPrimType>(R.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::EulerAnglesRPY<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::EulerAnglesRPY<SourcePrimType> & rpy) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(getYPRFromRPY<SourcePrimType, DestPrimType>(rpy.toImplementation()));
  }
};

template<typename DestPrimType, typename SourcePrimType>
class ConversionTraits<eigen_implementation::EulerAnglesYPR<DestPrimType>, eigen_implementation::EulerAnglesYPR<SourcePrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<DestPrimType> convert(const eigen_implementation::EulerAnglesYPR<SourcePrimType> & ypr) {
    return eigen_implementation::EulerAnglesYPR<DestPrimType>(ypr.toImplementation().template cast<DestPrimType>());
  }
};


template<typename PrimType>
class MultiplicationTraits<eigen_implementation::AngleAxis<PrimType>, eigen_implementation::AngleAxis<PrimType>> {
public:
  inline static eigen_implementation::AngleAxis<PrimType> mult(const eigen_implementation::AngleAxis<PrimType> & a, const eigen_implementation::AngleAxis<PrimType> & b) {
    return eigen_implementation::AngleAxis<PrimType>(Eigen::AngleAxis<PrimType>(a.toImplementation()*b.toImplementation()));
  }
};

template<typename PrimType>
class MultiplicationTraits<eigen_implementation::EulerAnglesRPY<PrimType>, eigen_implementation::EulerAnglesRPY<PrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<PrimType> mult(const eigen_implementation::EulerAnglesRPY<PrimType> & a, const eigen_implementation::EulerAnglesRPY<PrimType> & b) {
    return a.toImplementation()*b.toImplementation();
  }
};

template<typename PrimType>
class MultiplicationTraits<eigen_implementation::EulerAnglesYPR<PrimType>, eigen_implementation::EulerAnglesYPR<PrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<PrimType> mult(const eigen_implementation::EulerAnglesYPR<PrimType> & a, const eigen_implementation::EulerAnglesRPY<PrimType> & b) {
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
class RotationTraits<eigen_implementation::RotationQuaternion<PrimType>> {
 public:
   inline static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::RotationQuaternion<PrimType> & r, const Eigen::Matrix<PrimType, 3, 1> & v){
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
