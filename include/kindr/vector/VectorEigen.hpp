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

#ifndef KINDR_PHYSICALQUANTITIES_VECTOR_HPP_
#define KINDR_PHYSICALQUANTITIES_VECTOR_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/vector/VectorBase.hpp"

namespace kindr {
namespace vector {
//! Implementation of rotations based on the C++ Eigen library
namespace eigen_impl {

/*! \class Vector
 * \brief Vector in n-dimensional-space.
 *
 * This class implements a vector in n-dimensional-space.
 * More precisely an interface to store and access the coordinates of a vector of a point in n-dimensional-space is provided.
 * \tparam PrimType_  Primitive type of the coordinates.
 * \tparam Dimension_  Dimension of the vector.
 * \ingroup vectors
 */
template<typename PrimType_, int Dimension_>
class Vector : public VectorBase<Vector<PrimType_, Dimension_> >, private Eigen::Matrix<PrimType_, Dimension_, 1> {
 private:
  /*! \brief The base type.
   */
  typedef VectorBase<Vector<PrimType_, Dimension_> > Base;
 public:
  /*! \brief The implementation type.
   *
   *  The implementation type is always an Eigen object.
   */
  typedef Eigen::Matrix<PrimType_, Dimension_, 1> Implementation;

  /*! \brief The primitive type of the coordinates.
   */
  typedef PrimType_ Scalar;

  /*! \brief The dimension of the vector.
   */
  static constexpr int Dimension = Dimension_;

  /*! \brief Default constructor initializes all coordinates of the vector with zero.
   */
  Vector()
    : Implementation(Implementation::Zero()) {
  }

  /*! \brief Constructor using other Vector.
   *  \param other   Vector<PrimTypeOther_, Dimension_>
   */
  template<typename PrimTypeOther_>
  Vector(const Vector<PrimTypeOther_, Dimension_>& other)
    : Implementation(other.toImplementation().template cast<PrimType_>()) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,Dimension_,1>
   */
  explicit Vector(const Implementation& other)
    : Implementation(other) {
  }

  /*! \brief Constructor using three scalars.
   *  \param x x-Component
   *  \param y y-Component
   *  \param z z-Component
   */
  template<int DimensionCopy_ = Dimension_>
  Vector(Scalar x, Scalar y, Scalar z, typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr)
    : Implementation(x,y,z) {
  }

  /*! \brief Sets all components of the vector to zero.
   * \returns reference
   */
  Vector<PrimType_, Dimension_>& setZero() {
    Implementation::setZero();
    return *this;
  }

  /*! \brief Set values.
   */
  using Implementation::operator<<;

  /*! \brief Get/set values.
   */
  using Implementation::operator();

  /*!\brief Get the head of the vector (copy)
   * \returns the head of the vector (copy)
   */
  template<int DimensionOutput_>
  Vector<PrimType_, DimensionOutput_> head() const {
    return Vector<PrimType_, DimensionOutput_>(this->toImplementation().head(DimensionOutput_));
  }

  /*!\brief Get the tail of the vector (copy)
   * \returns the tail of the vector (copy)
   */
  template<int DimensionOutput_>
  Vector<PrimType_, DimensionOutput_> tail() const {
    return Vector<PrimType_, DimensionOutput_>(this->toImplementation().tail(DimensionOutput_));
  }

  /*!\brief Get a block of the vector (copy)
   * \returns a block of the vector (copy)
   */
  template<int Start_, int DimensionOutput_>
  Vector<PrimType_, DimensionOutput_> segment() const {
    return Vector<PrimType_, DimensionOutput_>(this->toImplementation().block<DimensionOutput_,1>(Start_,0));
  }

  /*!\brief Get x-coordinate of the vector (copy)
   * \returns the x-coordinate of the vector (copy)
   */
  template<int DimensionCopy_ = Dimension_>
  Scalar x(typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) const {
    return this->toImplementation().x();
  }

  /*!\brief Get x-coordinate of the vector (reference)
   * \returns the x-coordinate of the vector (reference)
   */
  template<int DimensionCopy_ = Dimension_>
  Scalar& x(typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) {
    return this->toImplementation().x();
  }

  /*!\brief Get y-coordinate of the vector (copy)
   * \returns the y-coordinate of the vector (copy)
   */
  template<int DimensionCopy_ = Dimension_>
  Scalar y(typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) const {
    return this->toImplementation().y();
  }

  /*!\brief Get y-coordinate of the vector (reference)
   * \returns the y-coordinate of the vector (reference)
   */
  template<int DimensionCopy_ = Dimension_>
  Scalar& y(typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) {
    return this->toImplementation().y();
  }

  /*!\brief Get z-coordinate of the vector (copy)
   * \returns the z-coordinate of the vector (copy)
   */
  template<int DimensionCopy_ = Dimension_>
  Scalar z(typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) const {
    return this->toImplementation().z();
  }

  /*!\brief Get z-coordinate of the vector (reference)
   * \returns the z-coordinate of the vector (reference)
   */
  template<int DimensionCopy_ = Dimension_>
  Scalar& z(typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) {
    return this->toImplementation().z();
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation& toImplementation() {
    return static_cast<Implementation&>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation& toImplementation() const {
    return static_cast<const Implementation&>(*this);
  }

  /*! \brief Addition of two vectors.
   * \param other   other vector
   * \returns sum
   */
  Vector<PrimType_, Dimension_> operator+(const Vector<PrimType_, Dimension_>& other) const {
    return Vector<PrimType_, Dimension_>(this->toImplementation() + other.toImplementation());
  }

  /*! \brief Subtraction of two vectors.
   * \param other   other vector
   * \returns difference
   */
  Vector<PrimType_, Dimension_> operator-(const Vector<PrimType_, Dimension_>& other) const {
    return Vector<PrimType_, Dimension_>(this->toImplementation() - other.toImplementation());
  }

  /*! \brief Multiplies vector with a scalar.
   * \param factor   factor
   * \returns product
   */
  template<typename PrimTypeFactor_>
  Vector<PrimType_, Dimension_> operator*(PrimTypeFactor_ factor) const {
    return Vector<PrimType_, Dimension_>(this->toImplementation()*(PrimType_)factor);
  }

  /*! \brief Divides vector by a scalar.
   * \param divisor   divisor
   * \returns quotient
   */
  template<typename PrimTypeDivisor_>
  Vector<PrimType_, Dimension_> operator/(PrimTypeDivisor_ divisor) const {
    return Vector<PrimType_, Dimension_>(this->toImplementation()/(PrimType_)divisor);
  }

  /*! \brief Addition and assignment of two vectors.
   * \param other   other vector
   * \returns reference
   */
  Vector<PrimType_, Dimension_>& operator+=(const Vector<PrimType_, Dimension_>& other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  /*! \brief Subtraction and assignment of two vectors.
   * \param other   other vector
   * \returns reference
   */
  Vector<PrimType_, Dimension_>& operator-=(const Vector<PrimType_, Dimension_>& other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  /*! \brief Multiplication with a scalar and assignment.
   * \param factor   factor
   * \returns reference
   */
  template<typename PrimTypeFactor_>
  Vector<PrimType_, Dimension_>& operator*=(PrimTypeFactor_ factor) {
    this->toImplementation() *= (PrimType_)factor;
    return *this;
  }

  /*! \brief Division by a scalar and assignment.
   * \param divisor   divisor
   * \returns reference
   */
  template<typename PrimTypeDivisor_>
  Vector<PrimType_, Dimension_>& operator/=(PrimTypeDivisor_ divisor) {
    this->toImplementation() /= (PrimType_)divisor;
    return *this;
  }

  /*! \brief Comparison operator.
   * \param other   other vector
   * \returns true if equal
   */
  bool operator==(const Vector<PrimType_, Dimension_>& other) const {
    return this->toImplementation() == other.toImplementation();
  }

  /*! \brief Comparison operator.
   * \param other   other vector
   * \returns true if unequal
   */
  bool operator!=(const Vector<PrimType_, Dimension_>& other) const {
    return this->toImplementation() != other.toImplementation();
  }

  /*! \brief Comparison function.
   * \param other   other vector
   * \param tol   tolerance
   * \returns true if similar within tolerance
   */
  bool isSimilarTo(const Vector<PrimType_, Dimension_>& other, Scalar tol) const {
    if((*this - other).abs().max() < tol) {
      return true;
    } else {
      return false;
    }
  }

  /*! \brief Norm of the vector.
   *  \returns norm.
   */
  Scalar norm() {
    return this->toImplementation().norm();
  }

  /*! \brief Normalizes the vector.
   *  \returns reference.
   */
  Vector<PrimType_, Dimension_>& normalize() {
    this->toImplementation().normalize();
    return *this;
  }

  /*! \brief Get a normalized version of the vector.
   *  \returns normalized vector.
   */
  Vector<PrimType_, Dimension_> normalized() const {
    return Vector<PrimType_, Dimension_>(this->toImplementation().normalized());
  }

  /*! \brief Dot product with other vector.
   *  \param other   other vector
   *  \returns dot product.
   */
  Scalar dot(const Vector<PrimType_, Dimension_>& other) const {
    return this->toImplementation().dot(other.toImplementation());
  }

  /*! \brief Cross product with other vector.
   *  \param other   other vector
   *  \returns cross product.
   */
  template<int DimensionCopy_ = Dimension_>
  Vector<PrimType_, Dimension_> cross(const Vector<PrimType_, Dimension_>& other, typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) const {
    return Vector<PrimType_, Dimension_>(this->toImplementation().cross(other.toImplementation()));
  }

  /*! \brief Elementwise product with other vector.
   *  \param other   other vector
   *  \returns elementwise product.
   */
  Vector<PrimType_, Dimension_> elementwiseMultiplication(const Vector<PrimType_, Dimension_>& other) const {
    return Vector<PrimType_, Dimension_>(this->toImplementation().cwiseProduct(other.toImplementation()));
  }

  /*! \brief Elementwise product with other vector.
   *  \param other   other vector
   *  \returns elementwise product.
   */
  Vector<PrimType_, Dimension_> elementwiseDivision(const Vector<PrimType_, Dimension_>& other) const {
    return Vector<PrimType_, Dimension_>(this->toImplementation().cwiseQuotient(other.toImplementation()));
  }

  /*! \brief Absolute components.
   *  \returns absolute components.
   */
  Vector<PrimType_, Dimension_> abs() const {
    return Vector<PrimType_, Dimension_>(this->toImplementation().cwiseAbs());
  }

  /*! \brief Maximum of the components.
   *  \returns maximum.
   */
  Scalar max() const {
    return this->toImplementation().maxCoeff();
  }

  /*! \brief Minimum of the components.
   *  \returns minimum.
   */
  Scalar min() const {
    return this->toImplementation().minCoeff();
  }

  /*! \brief Sum of the components.
   *  \returns sum.
   */
  Scalar sum() const {
    return this->toImplementation().sum();
  }

  /*! \brief Mean of the components.
   *  \returns mean.
   */
  Scalar mean() const {
    return this->toImplementation().mean();
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream& operator << (std::ostream& out, const Vector<PrimType_, Dimension_>& vector) {
    out << vector.transpose();
    return out;
  }
};


/*! \brief Multiplies vector with a scalar.
 * \param factor   factor
 * \returns product
 */
template<typename PrimTypeFactor_, typename PrimType_, int Dimension_>
Vector<PrimType_, Dimension_> operator*(PrimTypeFactor_ factor, const Vector<PrimType_, Dimension_>& vector) {
  return vector*(PrimType_)factor;
}


//! \brief 1D-Vector with primitive type double
typedef Vector<double,1>  Vector1D;

//! \brief 1D-Vector with primitive type float
typedef Vector<float,1>  Vector1F;

//! \brief 2D-Vector with primitive type double
typedef Vector<double,2>  Vector2D;

//! \brief 2D-Vector with primitive type float
typedef Vector<float,2>  Vector2F;

//! \brief 3D-Vector with primitive type double
typedef Vector<double,3>  Vector3D;

//! \brief 3D-Vector with primitive type float
typedef Vector<float,3>  Vector3F;

//! \brief 4D-Vector with primitive type double
typedef Vector<double,4>  Vector4D;

//! \brief 4D-Vector with primitive type float
typedef Vector<float,4>  Vector4F;

//! \brief 5D-Vector with primitive type double
typedef Vector<double,5>  Vector5D;

//! \brief 5D-Vector with primitive type float
typedef Vector<float,5>  Vector5F;

//! \brief 6D-Vector with primitive type double
typedef Vector<double,6>  Vector6D;

//! \brief 6D-Vector with primitive type float
typedef Vector<float,6>  Vector6F;

} // namespace eigen_impl



namespace internal {

/*! \brief Gets the primitive type of the vector
 */
template<typename PrimType_, int Dimension_>
class get_scalar<eigen_impl::Vector<PrimType_, Dimension_>> {
 public:
  typedef PrimType_ Scalar;
};

/*! \brief Gets the dimension of the vector
 */
template<typename PrimType_, int Dimension_>
class get_dimension<eigen_impl::Vector<PrimType_, Dimension_>> {
 public:
  static constexpr int Dimension = Dimension_;
};

} // namespace internal


} // namespace vector
} // namespace kindr





#endif /* KINDR_PHYSICALQUANTITIES_VECTOR_HPP_ */
