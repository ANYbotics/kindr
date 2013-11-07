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

  AngleAxis()
    : Base(Base::Identity()) {
  }

  AngleAxis(const PrimType & chi, const PrimType & v1, const PrimType & v2, const PrimType & v3)
    : Base(chi,Eigen::Matrix<PrimType,3,1>(v1,v2,v3)) {
  }

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
    *this = internal::ConversionTraits<AngleAxis, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other)); // todo: ok?
    return *this;
  }

//  AngleAxis<PrimType> & operator =(const RotationQuaternion<PrimType> & other) {
//    Base(internal::ConversionTraits<AngleAxis<PrimType>, RotationQuaternion<PrimType> >::convert(RotationQuaternion<PrimType>));
//    return *this;
//  }

  AngleAxis inverse() {
    return AngleAxis(Base::inverse());
  }

  inline Base & toImplementation() {
    return static_cast<Base &>(*this);
  }
  inline const Base & toImplementation() const {
    return static_cast<const Base &>(*this);
  }

  friend std::ostream & operator << (std::ostream & out, const AngleAxis & a) {
    out << a.toImplementation().angle() << ", " << a.toImplementation().axis().transpose();
    return out;
  }
};

typedef AngleAxis<double> AngleAxisD;
typedef AngleAxis<float> AngleAxisF;



template<typename PrimType>
class RotationQuaternion : public RotationQuaternionBase<RotationQuaternion<PrimType>>, public quaternions::eigen_implementation::UnitQuaternion<PrimType> {
 private:
  typedef quaternions::eigen_implementation::UnitQuaternion<PrimType> Base;
 public:
  typedef typename Base::Implementation::Implementation Implementation; // TODO: ok?
  typedef PrimType Scalar;

//  typedef typename internal::get_vector3<RotationQuaternion>::type Vector3;

  RotationQuaternion()
    : Base(Implementation::Identity()) { // todo: difference between base and implementation?
  }

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
    : Base(internal::ConversionTraits<RotationQuaternion, DERIVED>::convert(static_cast<const DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  RotationQuaternion & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other)); // todo: ok?
    return *this;
  }

//  template<typename DERIVED>
//  inline RotationQuaternion(const Rotation<DERIVED> & other)
//      : Base(internal::ConversionTraits<RotationQuaternion, DERIVED>::convert(other)) {
//  }
//
//  template<typename OTHER_DERIVED>
//  RotationQuaternion & operator =(const Rotation<OTHER_DERIVED> & other) {
//    Base(internal::ConversionTraits<RotationQuaternion, OTHER_DERIVED>::convert(other)); // todo: ok?
//    return *this;
//  }

  RotationQuaternion inverse() {
    return RotationQuaternion(Base::inverse());
  }

  RotationQuaternion conjugate() {
    return RotationQuaternion(Base::conjugate());
  }

//  using Base::inverse; todo: if this is used, the result is a unitquaternion, which cannot be further used

//  Vector<3> imag();

  using Base::toImplementation;

  friend std::ostream & operator << (std::ostream & out, const RotationQuaternion & quat) {
    out << quat.toImplementation().w() << " " << quat.toImplementation().x() << " " << quat.toImplementation().y() << " " << quat.toImplementation().z();
    return out;
  }
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

  RotationMatrix()
    : Base(Base::Identity()) {
  }

  RotationMatrix(const PrimType & r11, const PrimType & r12, const PrimType & r13,
                 const PrimType & r21, const PrimType & r22, const PrimType & r23,
                 const PrimType & r31, const PrimType & r32, const PrimType & r33) {
    *this << r11,r12,r13,r21,r22,r23,r31,r32,r33;
  }

  // create from Eigen::Matrix
  explicit RotationMatrix(const Base & other)
      : Base(other) {
  }

  // create from other rotation
  template<typename DERIVED>
  inline RotationMatrix(const Rotation<DERIVED> & other) // TODO not explicit anymore
      : Base(internal::ConversionTraits<RotationMatrix, DERIVED>::convert(static_cast<const DERIVED &>(other))) {
  }

  template<typename OTHER_DERIVED>
  RotationMatrix & operator =(const Rotation<OTHER_DERIVED> & other) {
    *this = internal::ConversionTraits<RotationMatrix, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other)); // todo: ok?
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

//  using Implementation::operator ==;

  template<typename OTHER_DERIVED> // todo ambiguous overload with Eigen operator if not specified
  bool operator ==(const Rotation<OTHER_DERIVED> & b) {
    return internal::ComparisonTraits<RotationMatrix>::isequal(*this, b);
  }

  friend std::ostream & operator << (std::ostream & out, const RotationMatrix & R) {
    out << R.toImplementation();
    return out;
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

  EulerAnglesRPY()
    : Base(Base::Zero()) {
  }

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
    *this = internal::ConversionTraits<EulerAnglesRPY, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other)); // todo: ok?
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

//  using Implementation::operator ==;

  template<typename OTHER_DERIVED> // todo ambiguous overload with Eigen operator if not specified
  bool operator ==(const Rotation<OTHER_DERIVED> & b) {
    return internal::ComparisonTraits<EulerAnglesRPY>::isequal(*this, b);
  }

  friend std::ostream & operator << (std::ostream & out, const EulerAnglesRPY & rpy) {
    out << rpy.toImplementation().transpose();
    return out;
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

  EulerAnglesYPR()
    : Base(Base::Zero()) {
  }

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
    *this = internal::ConversionTraits<EulerAnglesYPR, OTHER_DERIVED>::convert(static_cast<const OTHER_DERIVED &>(other)); // todo: ok?
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

//  using Implementation::operator ==;

  template<typename OTHER_DERIVED> // todo ambiguous overload with Eigen operator if not specified
  bool operator ==(const Rotation<OTHER_DERIVED> & b) {
    return internal::ComparisonTraits<EulerAnglesYPR>::isequal(*this, b);
  }

  friend std::ostream & operator << (std::ostream & out, const EulerAnglesYPR & ypr) {
    out << ypr.toImplementation().transpose();
    return out;
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
  return RotationQuaternion<PrimType> (quaternions::internal::MultiplicationTraits<RotationQuaternion<PrimType>, RotationQuaternion<PrimType>>::mult(a, b));
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


//template<typename PrimType>
//bool operator ==(const AngleAxis<PrimType> & a, const AngleAxis<PrimType> & b) {
////  return RotationMatrix<PrimType>(a).toImplementation() == RotationMatrix<PrimType>(b).toImplementation();
//  return internal::ComparisonTraits<AngleAxis<PrimType>>::isequal(a, b);
//}
//
//template<typename PrimType>
//bool operator ==(const RotationQuaternion<PrimType> & a, const RotationQuaternion<PrimType> & b) {
//  return RotationMatrix<PrimType>(a).toImplementation() == RotationMatrix<PrimType>(b).toImplementation();
//}
//
//template<typename PrimType> // ambiguous overload with Eigen operator if not specified
//bool operator ==(const RotationMatrix<PrimType> & a, const RotationMatrix<PrimType> & b) {
//  return a.toImplementation() == b.toImplementation();
//}
//
//template<typename PrimType> // ambiguous overload with Eigen operator if not specified
//bool operator ==(const EulerAnglesRPY<PrimType> & a, const EulerAnglesRPY<PrimType> & b) {
//  return a.toImplementation() == b.toImplementation();
//}
//
//template<typename PrimType> // ambiguous overload with Eigen operator if not specified
//bool operator ==(const EulerAnglesYPR<PrimType> & a, const EulerAnglesYPR<PrimType> & b) {
//  return a.toImplementation() == b.toImplementation();
//}





}  // namespace eigen_implementation


namespace internal {

// vector
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

//// matrix
//template<typename PrimType>
//class get_matrix3X<eigen_implementation::AngleAxis<PrimType>>{
// public:
//  typedef Eigen::Matrix<PrimType, 3, Eigen::Dynamic> type;
//};
//
//template<typename PrimType>
//class get_matrix3X<eigen_implementation::RotationQuaternion<PrimType>>{
// public:
//  typedef Eigen::Matrix<PrimType, 3, Eigen::Dynamic> type;
//};
//
//template<typename PrimType>
//class get_matrix3X<eigen_implementation::RotationMatrix<PrimType>>{
// public:
//  typedef Eigen::Matrix<PrimType, 3, Eigen::Dynamic> type;
//};
//
//template<typename PrimType>
//class get_matrix3X<eigen_implementation::EulerAnglesRPY<PrimType>>{
// public:
//  typedef Eigen::Matrix<PrimType, 3, Eigen::Dynamic> type;
//};
//
//template<typename PrimType>
//class get_matrix3X<eigen_implementation::EulerAnglesYPR<PrimType>>{
// public:
//  typedef Eigen::Matrix<PrimType, 3, Eigen::Dynamic> type;
//};


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


//template<typename PrimType>
//class MultiplicationTraits<eigen_implementation::AngleAxis<PrimType>, eigen_implementation::AngleAxis<PrimType>> {
//public:
//  inline static eigen_implementation::AngleAxis<PrimType> mult(const eigen_implementation::AngleAxis<PrimType> & a, const eigen_implementation::AngleAxis<PrimType> & b) {
//    return eigen_implementation::AngleAxis<PrimType>(Eigen::AngleAxis<PrimType>(a.toImplementation()*b.toImplementation()));
//  }
//};

template<typename PrimType>
class MultiplicationTraits<eigen_implementation::EulerAnglesRPY<PrimType>, eigen_implementation::EulerAnglesRPY<PrimType>> {
public:
  inline static eigen_implementation::EulerAnglesRPY<PrimType> mult(const eigen_implementation::EulerAnglesRPY<PrimType> & a, const eigen_implementation::EulerAnglesRPY<PrimType> & b) {
    return eigen_implementation::EulerAnglesRPY<PrimType>(eigen_implementation::RotationQuaternion<PrimType>(eigen_implementation::RotationQuaternion<PrimType>(a).toImplementation()*eigen_implementation::RotationQuaternion<PrimType>(b).toImplementation()));
  }
};

template<typename PrimType>
class MultiplicationTraits<eigen_implementation::EulerAnglesYPR<PrimType>, eigen_implementation::EulerAnglesYPR<PrimType>> {
public:
  inline static eigen_implementation::EulerAnglesYPR<PrimType> mult(const eigen_implementation::EulerAnglesYPR<PrimType> & a, const eigen_implementation::EulerAnglesYPR<PrimType> & b) {
    return eigen_implementation::EulerAnglesYPR<PrimType>(eigen_implementation::RotationQuaternion<PrimType>(eigen_implementation::RotationQuaternion<PrimType>(a).toImplementation()*eigen_implementation::RotationQuaternion<PrimType>(b).toImplementation()));
  }
};

// vector
template<typename PrimType> // todo: replace with standard function
class RotationTraits<eigen_implementation::AngleAxis<PrimType>> {
 public:
   inline static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::AngleAxis<PrimType> & aa, const Eigen::Matrix<PrimType, 3, 1> & v){
     return aa.toImplementation() * v;
   }
};

template<typename PrimType>
class RotationTraits<eigen_implementation::RotationQuaternion<PrimType>> {
 public:
   inline static typename Eigen::Matrix<PrimType, 3, 1> rotate(const eigen_implementation::RotationQuaternion<PrimType> & p, const Eigen::Matrix<PrimType, 3, 1> & v){
     return p.toImplementation() * v;
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

//// matrix
//template<typename PrimType>
//class RotationTraits<eigen_implementation::AngleAxis<PrimType>> {
// public:
//   inline static typename Eigen::Matrix<PrimType, 3, Eigen::Dynamic> rotate(const eigen_implementation::AngleAxis<PrimType> & aa, const Eigen::Matrix<PrimType, 3, Eigen::Dynamic> & m){
//     return aa.toImplementation() * m;
//   }
//};
//
//template<typename PrimType>
//class RotationTraits<eigen_implementation::RotationQuaternion<PrimType>> {
// public:
//   inline static typename Eigen::Matrix<PrimType, 3, Eigen::Dynamic> rotate(const eigen_implementation::RotationQuaternion<PrimType> & p, const Eigen::Matrix<PrimType, 3, Eigen::Dynamic> & m){
//     return p.toImplementation() * m;
//   }
//};
//
//template<typename PrimType>
//class RotationTraits<eigen_implementation::RotationMatrix<PrimType>> {
// public:
//   inline static typename Eigen::Matrix<PrimType, 3, Eigen::Dynamic> rotate(const eigen_implementation::RotationMatrix<PrimType> & R, const Eigen::Matrix<PrimType, 3, Eigen::Dynamic> & m){
//     return R.toImplementation() * m;
//   }
//};
//
//template<typename PrimType>
//class RotationTraits<eigen_implementation::EulerAnglesRPY<PrimType>> {
// public:
//   inline static typename Eigen::Matrix<PrimType, 3, Eigen::Dynamic> rotate(const eigen_implementation::EulerAnglesRPY<PrimType> & rpy, const Eigen::Matrix<PrimType, 3, Eigen::Dynamic> & m){
//     return eigen_implementation::RotationMatrix<PrimType>(rpy).toImplementation() * m;
//   }
//};
//
//template<typename PrimType>
//class RotationTraits<eigen_implementation::EulerAnglesYPR<PrimType>> {
// public:
//   inline static typename Eigen::Matrix<PrimType, 3, Eigen::Dynamic> rotate(const eigen_implementation::EulerAnglesYPR<PrimType> & ypr, const Eigen::Matrix<PrimType, 3, Eigen::Dynamic> & m){
//     return eigen_implementation::RotationMatrix<PrimType>(ypr).toImplementation() * m;
//   }
//};



template<typename PrimType>
class ComparisonTraits<eigen_implementation::AngleAxis<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::AngleAxis<PrimType> & a, const eigen_implementation::AngleAxis<PrimType> & b){
     return (a.toImplementation().angle() ==  b.toImplementation().angle() && a.toImplementation().axis() ==  b.toImplementation().axis()) ||
            (a.toImplementation().angle() == -b.toImplementation().angle() && a.toImplementation().axis() == -b.toImplementation().axis());
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::RotationQuaternion<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::RotationQuaternion<PrimType> & a, const eigen_implementation::RotationQuaternion<PrimType> & b){
     return (a.toImplementation().w() ==  b.toImplementation().w() &&
             a.toImplementation().x() ==  b.toImplementation().x() &&
             a.toImplementation().y() ==  b.toImplementation().y() &&
             a.toImplementation().z() ==  b.toImplementation().z()) ||
            (a.toImplementation().w() == -b.toImplementation().w() &&
             a.toImplementation().x() == -b.toImplementation().x() &&
             a.toImplementation().y() == -b.toImplementation().y() &&
             a.toImplementation().z() == -b.toImplementation().z());
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::RotationMatrix<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::RotationMatrix<PrimType> & a, const eigen_implementation::RotationMatrix<PrimType> & b){
     return a.toImplementation() == b.toImplementation();
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::EulerAnglesRPY<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::EulerAnglesRPY<PrimType> & a, const eigen_implementation::EulerAnglesRPY<PrimType> & b){
     return a.toImplementation() == b.toImplementation();
   }
};

template<typename PrimType>
class ComparisonTraits<eigen_implementation::EulerAnglesYPR<PrimType>> {
 public:
   inline static bool isequal(const eigen_implementation::EulerAnglesYPR<PrimType> & a, const eigen_implementation::EulerAnglesYPR<PrimType> & b){
     return a.toImplementation() == b.toImplementation();
   }
};



} // namespace internal
} // namespace rotations
} // namespace rm

#endif /* ROTATIONEIGEN_HPP_ */
