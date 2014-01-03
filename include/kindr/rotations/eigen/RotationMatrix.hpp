/*
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef KINDR_ROTATIONS_EIGEN_ROTATIONMATRIX_HPP_
#define KINDR_ROTATIONS_EIGEN_ROTATIONMATRIX_HPP_

#include <cmath>

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationBase.hpp"
#include "kindr/rotations/eigen/RotationEigenFunctions.hpp"

namespace kindr {
namespace rotations {
namespace eigen_impl {


/*! \class RotationMatrix
 *  \brief Implementation of matrix rotation based on Eigen::Matrix<Scalar, 3, 3>
 *
 *  The following four typedefs are provided for convenience:
 *   - \ref eigen_impl::RotationMatrixAD "RotationMatrixAD" for active rotation and double primitive type
 *   - \ref eigen_impl::RotationMatrixAF "RotationMatrixAF" for active rotation and float primitive type
 *   - \ref eigen_impl::RotationMatrixPD "RotationMatrixPD" for passive rotation and double primitive type
 *   - \ref eigen_impl::RotationMatrixPF "RotationMatrixPF" for passive rotation and float primitive type
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *  \tparam Usage_ the rotation usage which is either active or passive
 *
 *  \ingroup rotations
 */
template<typename PrimType_, enum RotationUsage Usage_>
class RotationMatrix : public RotationMatrixBase<RotationMatrix<PrimType_, Usage_>, Usage_>, private Eigen::Matrix<PrimType_, 3, 3> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType_, 3, 3> Base;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;
  /*! \brief The primitive type.
   *  Float/Double
   */
  typedef PrimType_ Scalar;

  /*! \brief Default constructor using identity rotation.
   */
  RotationMatrix()
    : Base(Base::Identity()) {
  }

  /*! \brief Constructor using nine scalars.
   *  In debug mode, an assertion is thrown if the matrix is not a rotation matrix.
   *  \param r11     entry in row 1, col 1
   *  \param r12     entry in row 1, col 2
   *  \param r13     entry in row 1, col 3
   *  \param r21     entry in row 2, col 1
   *  \param r22     entry in row 2, col 2
   *  \param r23     entry in row 2, col 3
   *  \param r31     entry in row 3, col 1
   *  \param r32     entry in row 3, col 2
   *  \param r33     entry in row 3, col 3
   */
  RotationMatrix(Scalar r11, Scalar r12, Scalar r13,
                 Scalar r21, Scalar r22, Scalar r23,
                 Scalar r31, Scalar r32, Scalar r33) {
    *this << r11,r12,r13,r21,r22,r23,r31,r32,r33;
    KINDR_ASSERT_MATRIX_NEAR_DBG(std::runtime_error, this->toImplementation() * this->toImplementation().transpose(), Base::Identity(), static_cast<Scalar>(1e-4), "Input matrix is not orthogonal.");
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, this->determinant(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input matrix determinant is not 1.");
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  In debug mode, an assertion is thrown if the rotation vector has not unit length.
   *  \param other   Eigen::Matrix<PrimType_,3,3>
   */
  explicit RotationMatrix(const Base& other)
  : Base(other) {
    KINDR_ASSERT_MATRIX_NEAR_DBG(std::runtime_error, other * other.transpose(), Base::Identity(), static_cast<Scalar>(1e-4), "Input matrix is not orthogonal.");
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, other.determinant(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input matrix determinant is not 1.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit RotationMatrix(const RotationBase<OtherDerived_, Usage_>& other)
    : Base(internal::ConversionTraits<RotationMatrix, OtherDerived_>::convert(other.derived()).toStoredImplementation()) {
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OtherDerived_>
  RotationMatrix& operator =(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toStoredImplementation() = internal::ConversionTraits<RotationMatrix, OtherDerived_>::convert(other.derived()).toStoredImplementation();
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationMatrix& operator ()(const RotationBase<OtherDerived_, Usage_>& other) {
    this->toStoredImplementation() = internal::ConversionTraits<RotationMatrix, OtherDerived_>::convert(other.derived()).toStoredImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  RotationMatrix inverted() const {
    return RotationMatrix(toStoredImplementation().transpose());
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  RotationMatrix& invert() {
    *this = RotationMatrix(toStoredImplementation().transpose());
    return *this;
  }

  /*! \brief Returns the transpose of the rotation matrix.
   *  \returns the inverse of the rotation
   */
  RotationMatrix transposed() const {
    return RotationMatrix(toStoredImplementation().transpose());
  }

  /*! \brief Transposes the rotation matrix.
   *  \returns reference
   */
  RotationMatrix& transpose() {
    *this = RotationMatrix(toStoredImplementation().transpose());
    return *this;
  }

  /*! \brief Returns the determinant of the rotation matrix.
   *  \returns determinant of the rotation matrix
   */
  Scalar determinant() const {
  return toStoredImplementation().determinant();
  }

  /*! \brief Returns the type used for the implementation.
   *  \returns the type used for the implementation
   */
  Implementation toImplementation() const {
    return toStoredImplementation();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation& toStoredImplementation() {
    return static_cast<Implementation&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation& toStoredImplementation() const {
    return static_cast<const Implementation&>(*this);
  }

  /*! \brief Reading access to the rotation matrix.
   *  \returns rotation matrix (matrix) with reading access
   */
  inline const Implementation& matrix() const {
    return toImplementation();
  }

  /*! \brief  Reading access to the rotation matrix.
   *  \returns rotation matrix (matrix) with reading access
   */
  inline Implementation matrix() {
    return toImplementation();
  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  RotationMatrix& setIdentity() {
    this->Implementation::setIdentity();
    return *this;
  }

  /*! \brief Returns a unique matrix rotation.
   *  A rotation matrix is always unique.
   *  This function is used to compare different rotations.
   *  \returns copy of the matrix rotation which is unique
   */
  RotationMatrix getUnique() const {
    return *this;
  }

  /*! \brief Modifies the matrix rotation such that it becomes unique.
   *  A rotation matrix is always unique.
   *  \returns reference
   */
  RotationMatrix& setUnique() {
    return *this;
  }

  /*! \brief Concenation operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator*.
   *  \returns the concenation of two rotations
   */
  using RotationMatrixBase<RotationMatrix<PrimType_, Usage_>, Usage_>::operator*; // otherwise ambiguous RotationBase and Eigen

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator==.
   *  \returns true if two rotations are similar.
   */
  using RotationMatrixBase<RotationMatrix<PrimType_, Usage_>, Usage_>::operator==; // otherwise ambiguous RotationBase and Eigen

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationMatrix& rotationMatrix) {
    out << rotationMatrix.toImplementation();
    return out;
  }
};

//! \brief Active matrix rotation with double primitive type
typedef RotationMatrix<double, RotationUsage::ACTIVE>  RotationMatrixAD;
//! \brief Active matrix rotation with float primitive type
typedef RotationMatrix<float,  RotationUsage::ACTIVE>  RotationMatrixAF;
//! \brief Passive matrix rotation with double primitive type
typedef RotationMatrix<double, RotationUsage::PASSIVE> RotationMatrixPD;
//! \brief Passive matrix rotation with float primitive type
typedef RotationMatrix<float,  RotationUsage::PASSIVE> RotationMatrixPF;

} // namespace eigen_impl


namespace internal {

template<typename PrimType_, enum RotationUsage Usage_>
class get_scalar<eigen_impl::RotationMatrix<PrimType_, Usage_>> {
 public:
  typedef PrimType_ Scalar;
};

template<typename PrimType_, enum RotationUsage Usage_>
class get_matrix3X<eigen_impl::RotationMatrix<PrimType_, Usage_>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>> {
 public:
  typedef eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE> OtherUsage;
};

template<typename PrimType_>
class get_other_usage<eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>> {
 public:
  typedef eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE> OtherUsage;
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationMatrix<DestPrimType_, Usage_>, eigen_impl::AngleAxis<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationMatrix<DestPrimType_, Usage_> convert(const eigen_impl::AngleAxis<SourcePrimType_, Usage_>& aa) {
    return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(eigen_impl::getRotationMatrixFromAngleAxis<SourcePrimType_, DestPrimType_>(aa.toStoredImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationMatrix<DestPrimType_, Usage_>, eigen_impl::RotationVector<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationMatrix<DestPrimType_, Usage_> convert(const eigen_impl::RotationVector<SourcePrimType_, Usage_>& rotationVector) {
    typename eigen_impl::RotationMatrix<DestPrimType_, Usage_>::Implementation matrix;
    typedef typename eigen_impl::RotationVector<SourcePrimType_, Usage_>::Scalar Scalar;
    const  typename eigen_impl::RotationVector<DestPrimType_, Usage_>::Implementation rv = rotationVector.toStoredImplementation().template cast<DestPrimType_>();
    const SourcePrimType_ v1 = rv.x();
    const SourcePrimType_ v2 = rv.y();
    const SourcePrimType_ v3 = rv.z();
    const SourcePrimType_ v = rv.norm();

    if (v < common::NumTraits<Scalar>::dummy_precision())  {
      matrix << 1.0,  v3, -v2,
                        -v3, 1.0,  v1,
                          v2, -v1, 1.0;
    } else {
      const DestPrimType_ t3 = v*(1.0/2.0);
      const DestPrimType_ t2 = sin(t3);
      const DestPrimType_ t4 = cos(t3);
      const DestPrimType_ t5 = 1.0/(v*v);
      const DestPrimType_ t6 = t4*v*v3;
      const DestPrimType_ t7 = t2*v1*v2;
      const DestPrimType_ t8 = t2*t2;
      const DestPrimType_ t9 = v1*v1;
      const DestPrimType_ t10 = v2*v2;
      const DestPrimType_ t11 = v3*v3;
      const DestPrimType_ t12 = v*v;
      const DestPrimType_ t13 = t4*t4;
      const DestPrimType_ t14 = t12*t13;
      const DestPrimType_ t15 = t2*v1*v3;
      const DestPrimType_ t16 = t4*v*v1;
      const DestPrimType_ t17 = t2*v2*v3;
      matrix(0,0) = t5*(t14-t8*(-t9+t10+t11));
      matrix(1,0) = t2*t5*(t6+t7)*2.0;
      matrix(2,0) = t2*t5*(t15-t4*v*v2)*2.0;
      matrix(0,1) = t2*t5*(t6-t7)*-2.0;
      matrix(1,1) = t5*(t14-t8*(t9-t10+t11));
      matrix(2,1) = t2*t5*(t16+t17)*2.0;
      matrix(0,2) = t2*t5*(t15+t4*v*v2)*2.0;
      matrix(1,2) = t2*t5*(t16-t17)*-2.0;
      matrix(2,2) = t5*(t14-t8*(t9+t10-t11));

    }
    return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(matrix);
    // the same as above:
//    return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(eigen_impl::AngleAxis<SourcePrimType_, Usage_>(rotationVector));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationMatrix<DestPrimType_, Usage_>, eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationMatrix<DestPrimType_, Usage_> convert(const eigen_impl::RotationQuaternion<SourcePrimType_, Usage_>& q) {
//    if (Usage_ == RotationUsage::ACTIVE) {
//      return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(eigen_impl::getRotationMatrixFromQuaternion<SourcePrimType_, DestPrimType_>(q.toImplementation()));
//    } if (Usage_ == RotationUsage::PASSIVE) {
//      return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(eigen_impl::getRotationMatrixFromQuaternion<SourcePrimType_, DestPrimType_>(q.toImplementation()).inverse());
//    }
    return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(eigen_impl::getRotationMatrixFromQuaternion<SourcePrimType_, DestPrimType_>(q.toStoredImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationMatrix<DestPrimType_, Usage_>, eigen_impl::RotationMatrix<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationMatrix<DestPrimType_, Usage_> convert(const eigen_impl::RotationMatrix<SourcePrimType_, Usage_>& R) {
    return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(R.toStoredImplementation().template cast<DestPrimType_>());
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationMatrix<DestPrimType_, Usage_>, eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationMatrix<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesXyz<SourcePrimType_, Usage_>& xyz) {
    return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(eigen_impl::getRotationMatrixFromRpy<SourcePrimType_, DestPrimType_>(xyz.toStoredImplementation()));
  }
};

template<typename DestPrimType_, typename SourcePrimType_, enum RotationUsage Usage_>
class ConversionTraits<eigen_impl::RotationMatrix<DestPrimType_, Usage_>, eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>> {
 public:
  inline static eigen_impl::RotationMatrix<DestPrimType_, Usage_> convert(const eigen_impl::EulerAnglesZyx<SourcePrimType_, Usage_>& zyx) {
    return eigen_impl::RotationMatrix<DestPrimType_, Usage_>(eigen_impl::getRotationMatrixFromYpr<SourcePrimType_, DestPrimType_>(zyx.toStoredImplementation()));
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Multiplication Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
/*! \brief Multiplication of a rotation matrix (on left hand side) and a rotation with a different parameterization (on right hand side)
 */
template<typename LeftPrimType_, typename Right_, enum RotationUsage Usage_>
class MultiplicationTraits<RotationBase<eigen_impl::RotationMatrix<LeftPrimType_, Usage_>, Usage_>, RotationBase<Right_, Usage_>> {
 public:
  inline static eigen_impl::RotationMatrix<LeftPrimType_, Usage_> mult(const eigen_impl::RotationMatrix<LeftPrimType_, Usage_>& lhs, const RotationBase<Right_, Usage_>& rhs) {
    return eigen_impl::RotationMatrix<LeftPrimType_, Usage_>(
               typename eigen_impl::RotationQuaternion<LeftPrimType_, Usage_>(
                   (typename eigen_impl::RotationQuaternion<LeftPrimType_,  Usage_>(lhs)).toImplementation() *
                   (typename eigen_impl::RotationQuaternion<typename get_scalar<Right_>::Scalar, Usage_>(rhs.derived())).toImplementation()
               )
           );
  }
};
/*! \brief Multiplication of a rotation matrix (on right hand side) and a rotation with a different parameterization (on left hand side)
 */
template<typename Left_, typename RightPrimType_ , enum RotationUsage Usage_>
class MultiplicationTraits<RotationBase<Left_, Usage_>,RotationBase<eigen_impl::RotationMatrix<RightPrimType_, Usage_>, Usage_>> {
 public:
  inline static Left_ mult( const RotationBase<Left_, Usage_>& lhs, const eigen_impl::RotationMatrix<RightPrimType_, Usage_>& rhs) {
    return Left_(
            eigen_impl::RotationMatrix<typename get_scalar<Left_>::Scalar, Usage_>(
                eigen_impl::RotationMatrix<RightPrimType_, Usage_>(lhs.derived()).toImplementation() *
                rhs.toImplementation()
            )
           );
  }
};

/*! \brief Multiplication of two rotation matrices
 */
template<typename PrimType_, enum RotationUsage Usage_>
class MultiplicationTraits<RotationBase<eigen_impl::RotationMatrix<PrimType_, Usage_>, Usage_>, RotationBase<eigen_impl::RotationMatrix<PrimType_, Usage_>, Usage_>> {
 public:
  inline static eigen_impl::RotationMatrix<PrimType_, Usage_> mult(const eigen_impl::RotationMatrix<PrimType_, Usage_>& lhs, const eigen_impl::RotationMatrix<PrimType_, Usage_>& rhs) {
    return  eigen_impl::RotationMatrix<PrimType_, Usage_>(lhs.toImplementation() * rhs.toImplementation());
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename PrimType_, enum RotationUsage Usage_>
class RotationTraits<eigen_impl::RotationMatrix<PrimType_, Usage_>> {
 public:
  template<typename get_matrix3X<eigen_impl::RotationMatrix<PrimType_, Usage_>>::IndexType Cols>
  inline static typename get_matrix3X<eigen_impl::RotationMatrix<PrimType_, Usage_>>::template Matrix3X<Cols> rotate(const eigen_impl::RotationMatrix<PrimType_, Usage_>& R, const typename get_matrix3X<eigen_impl::RotationMatrix<PrimType_, Usage_>>::template Matrix3X<Cols>& m){
    return R.toStoredImplementation() * m;
  }
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Comparison Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Usage Conversion Traits - required?
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename PrimType_>
class UsageConversionTraits<eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>,RotationUsage::PASSIVE> {
 public:
  inline static typename get_other_usage<eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>>::OtherUsage getActive(const eigen_impl::RotationMatrix<PrimType_,RotationUsage::PASSIVE>& in) {
    return typename get_other_usage<eigen_impl::RotationMatrix<PrimType_, RotationUsage::PASSIVE>>::OtherUsage(in.toImplementation());
  }

  // getPassive() does not exist (on purpose)
};

template<typename PrimType_>
class UsageConversionTraits<eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>,RotationUsage::ACTIVE> {
 public:
  inline static typename get_other_usage<eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>>::OtherUsage getPassive(const eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>& in) {
    return typename get_other_usage<eigen_impl::RotationMatrix<PrimType_, RotationUsage::ACTIVE>>::OtherUsage(in.toImplementation());
  }

  // getActive() does not exist (on purpose)
};

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Box Operations - required?
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

template<typename LeftPrimType_, typename RightPrimType_, enum RotationUsage Usage_>
class BoxOperationTraits<RotationBase<eigen_impl::RotationMatrix<LeftPrimType_, Usage_>, Usage_>, RotationBase<eigen_impl::RotationMatrix<RightPrimType_, Usage_>, Usage_>> {
 public:
  inline static typename internal::get_matrix3X<eigen_impl::RotationMatrix<LeftPrimType_, Usage_>>::template Matrix3X<1> boxMinus(const eigen_impl::RotationMatrix<LeftPrimType_, Usage_>& lhs, const eigen_impl::RotationMatrix<RightPrimType_, Usage_>& rhs) {
    return (lhs*rhs.inverted()).getLogarithmicMap();
  }

  inline static  eigen_impl::RotationMatrix<LeftPrimType_, Usage_> boxPlus(const eigen_impl::RotationMatrix<RightPrimType_, Usage_>& rotation, const typename internal::get_matrix3X<eigen_impl::RotationMatrix<RightPrimType_, Usage_>>::template Matrix3X<1>& vector) {
    return eigen_impl::RotationMatrix<LeftPrimType_, Usage_>((MapTraits<RotationBase<eigen_impl::RotationMatrix<RightPrimType_,Usage_>, Usage_>>::exponentialMap(vector))*rotation);
  }
};


} // namespace internal
} // namespace rotations
} // namespace kindr


#endif /* KINDR_ROTATIONS_EIGEN_ROTATIONMATRIX_HPP_ */
