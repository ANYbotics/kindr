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

#pragma once

#include <cmath>

#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/rotations/RotationBase.hpp"

namespace kindr {


/*! \class RotationMatrix
 *  \brief Implementation of matrix rotation based on Eigen::Matrix<Scalar, 3, 3>
 *
 *  The following four typedefs are provided for convenience:
 *   - \ref RotationMatrixAD "RotationMatrixD" for double primitive type
 *   - \ref RotationMatrixAF "RotationMatrixF" for float primitive type
 *
 *  \tparam PrimType_ the primitive type of the data (double or float)
 *
 *  \ingroup rotations
 */
template<typename PrimType_>
class RotationMatrix : public RotationBase<RotationMatrix<PrimType_>>, private Eigen::Matrix<PrimType_, 3, 3> {
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
  // : Base(other)
  {

    this->toImplementation() = other;

    KINDR_ASSERT_MATRIX_NEAR_DBG(std::runtime_error, other * other.transpose(), Base::Identity(), static_cast<Scalar>(1e-4), "Input matrix is not orthogonal.");
    KINDR_ASSERT_SCALAR_NEAR_DBG(std::runtime_error, other.determinant(), static_cast<Scalar>(1), static_cast<Scalar>(1e-4), "Input matrix determinant is not 1.");
  }

  /*! \brief Constructor using another rotation.
   *  \param other   other rotation
   */
  template<typename OtherDerived_>
  inline explicit RotationMatrix(const RotationBase<OtherDerived_>& other)
  // : Base(internal::ConversionTraits<RotationMatrix, OtherDerived_>::convert(other.derived()).toImplementation())
  {
    this->toImplementation() = internal::ConversionTraits<RotationMatrix, OtherDerived_>::convert(other.derived()).toImplementation();
  }

  /*! \brief Assignment operator using another rotation.
   *  \param other   other rotation
   *  \returns referece
   */
  template<typename OtherDerived_>
  RotationMatrix& operator =(const RotationBase<OtherDerived_>& other) {
    this->toImplementation() = internal::ConversionTraits<RotationMatrix, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Parenthesis operator to convert from another rotation.
   *  \param other   other rotation
   *  \returns reference
   */
  template<typename OtherDerived_>
  RotationMatrix& operator ()(const RotationBase<OtherDerived_>& other) {
    this->toImplementation() = internal::ConversionTraits<RotationMatrix, OtherDerived_>::convert(other.derived()).toImplementation();
    return *this;
  }

  /*! \brief Returns the inverse of the rotation.
   *  \returns the inverse of the rotation
   */
  RotationMatrix inverted() const {
    RotationMatrix matrix;
    matrix.toImplementation() = this->toImplementation().transpose();
    return matrix;
  }

  /*! \brief Inverts the rotation.
   *  \returns reference
   */
  RotationMatrix& invert() {
    *this = this->inverted();
    return *this;
  }

  /*! \brief Returns the transpose of the rotation matrix.
   *  \returns the inverse of the rotation
   */
  RotationMatrix transposed() const {
    RotationMatrix matrix;
    matrix.toImplementation() = this->toImplementation().transpose();
    return matrix;
  }

  /*! \brief Transposes the rotation matrix.
   *  \returns reference
   */
  RotationMatrix& transpose() {
    *this = this->transposed();
    return *this;
  }

  /*! \brief Returns the determinant of the rotation matrix.
   *  \returns determinant of the rotation matrix
   */
  Scalar determinant() const {
  return toImplementation().determinant();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation for direct manipulation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return static_cast<const Implementation&>(*this);
  }

  /*! \brief Reading access to the rotation matrix.
   *  \returns rotation matrix (matrix) with reading access
   */
  inline Implementation matrix() const {
      return this->toImplementation();
  }

  /*! \brief  Writing access to the rotation matrix.
   */
  inline void setMatrix(const Implementation & input) {
      this->toImplementation() = input;
  }

  /*! \brief  Writing access to the rotation matrix.
   */
  inline void setMatrix(Scalar r11, Scalar r12, Scalar r13,
                        Scalar r21, Scalar r22, Scalar r23,
                        Scalar r31, Scalar r32, Scalar r33) {

     *this << r11,r12,r13,r21,r22,r23,r31,r32,r33;

  }

  /*! \brief Sets the rotation to identity.
   *  \returns reference
   */
  RotationMatrix& setIdentity() {
    this->Implementation::setIdentity();
    return *this;
  }

  /*! \brief Get identity rotation.
   *  \returns identity rotation
   */
  static RotationMatrix Identity() {
    return RotationMatrix(Base::Identity());
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
  using RotationBase<RotationMatrix<PrimType_>>::operator*; // otherwise ambiguous RotationBase and Eigen

  /*! \brief Equivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator==.
   *  \returns true if two rotations are equal.
   */
  using RotationBase<RotationMatrix<PrimType_>>::operator==; // otherwise ambiguous RotationBase and Eigen

  /*! \brief Inequivalence operator.
   *  This is explicitly specified, because Eigen::Matrix provides also an operator!=.
   *  \returns true if two rotations are not equal.
   */
  using RotationBase<RotationMatrix<PrimType_>>::operator!=; // otherwise ambiguous RotationBase and Eigen


  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const RotationMatrix& rotationMatrix) {
    out << rotationMatrix.toImplementation();
    return out;
  }
};

//! \brief Active matrix rotation with double primitive type
typedef RotationMatrix<double>  RotationMatrixD;
//! \brief Active matrix rotation with float primitive type
typedef RotationMatrix<float>  RotationMatrixF;
//! \brief Passive matrix rotation with double primitive type
typedef RotationMatrix<double> RotationMatrixPD;
//! \brief Passive matrix rotation with float primitive type
typedef RotationMatrix<float> RotationMatrixPF;



namespace internal {

template<typename PrimType_>
class get_scalar<RotationMatrix<PrimType_>> {
 public:
  typedef PrimType_ Scalar;
};

template<typename PrimType_>
class get_matrix3X<RotationMatrix<PrimType_>>{
 public:
  typedef int  IndexType;

  template <IndexType Cols>
  using Matrix3X = Eigen::Matrix<PrimType_, 3, Cols>;
};


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Conversion Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<RotationMatrix<DestPrimType_>, AngleAxis<SourcePrimType_>> {
 public:
  inline static RotationMatrix<DestPrimType_> convert(const AngleAxis<SourcePrimType_>& aa) {
    RotationMatrix<DestPrimType_> matrix;
    matrix.toImplementation() = (aa.toImplementation().template cast<DestPrimType_>()).toRotationMatrix();
    return matrix;
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<RotationMatrix<DestPrimType_>, RotationVector<SourcePrimType_>> {
 public:
  inline static RotationMatrix<DestPrimType_> convert(const RotationVector<SourcePrimType_>& rotationVector) {
    return RotationMatrix<DestPrimType_>(RotationQuaternion<DestPrimType_>(rotationVector));
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<RotationMatrix<DestPrimType_>, RotationQuaternion<SourcePrimType_>> {
 public:
  inline static RotationMatrix<DestPrimType_> convert(const RotationQuaternion<SourcePrimType_>& q) {
    RotationMatrix<DestPrimType_> matrix;
    matrix.toImplementation() = (q.toImplementation().template cast<DestPrimType_>()).toRotationMatrix();
    return matrix;
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<RotationMatrix<DestPrimType_>, RotationMatrix<SourcePrimType_>> {
 public:
  inline static RotationMatrix<DestPrimType_> convert(const RotationMatrix<SourcePrimType_>& R) {
    RotationMatrix<DestPrimType_> matrix;
    matrix.toImplementation() = R.toImplementation().template cast<DestPrimType_>();
    return matrix;
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<RotationMatrix<DestPrimType_>, EulerAnglesXyz<SourcePrimType_>> {
 public:
  inline static RotationMatrix<DestPrimType_> convert(const EulerAnglesXyz<SourcePrimType_>& xyz) {
      RotationMatrix<DestPrimType_> matrix;
      matrix.toImplementation() = RotationQuaternion<DestPrimType_>(xyz).toImplementation().toRotationMatrix();
      return matrix;
  }
};

template<typename DestPrimType_, typename SourcePrimType_>
class ConversionTraits<RotationMatrix<DestPrimType_>, EulerAnglesZyx<SourcePrimType_>> {
 public:
  inline static RotationMatrix<DestPrimType_> convert(const EulerAnglesZyx<SourcePrimType_>& zyx) {
    RotationMatrix<DestPrimType_> matrix;
    matrix.toImplementation() = RotationQuaternion<DestPrimType_>(zyx).toImplementation().toRotationMatrix();
    return matrix;
  }
};



/*! \brief Multiplication of two rotation matrices
 */
template<typename PrimType_>
class MultiplicationTraits<RotationBase<RotationMatrix<PrimType_>>, RotationBase<RotationMatrix<PrimType_>>> {
 public:
  inline static RotationMatrix<PrimType_> mult(const RotationMatrix<PrimType_>& lhs, const RotationMatrix<PrimType_>& rhs) {
      RotationMatrix<PrimType_> result;
      result.toImplementation() = lhs.toImplementation() * rhs.toImplementation();
      return result;
  }
};



/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Rotation Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Comparison Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Box Operations - required?
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */


/* -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Fixing Traits
 * ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- */
template<typename PrimType_>
class FixingTraits<RotationMatrix<PrimType_>> {
 public:
  inline static void fix(RotationMatrix<PrimType_>& R) {
    const PrimType_ factor = 1/pow(R.determinant(), 1.0/3.0);
    R.setMatrix(factor*R.matrix()(0,0),
                factor*R.matrix()(0,1),
                factor*R.matrix()(0,2),
                factor*R.matrix()(1,0),
                factor*R.matrix()(1,1),
                factor*R.matrix()(1,2),
                factor*R.matrix()(2,0),
                factor*R.matrix()(2,1),
                factor*R.matrix()(2,2));
  }
};


} // namespace internal
} // namespace kindr
