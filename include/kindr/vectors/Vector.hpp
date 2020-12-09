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

#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "kindr/common/common.hpp"
#include "kindr/common/assert_macros_eigen.hpp"
#include "kindr/vectors/VectorBase.hpp"
#include "kindr/phys_quant/PhysicalType.hpp"

namespace kindr {

/*! \class Vector
 * \brief Vector in n-dimensional-space.
 *
 * This class implements a vector in n-dimensional-space.
 * More precisely an interface to store and access the coordinates of a vector of a point in n-dimensional-space is provided.
 * \tparam PhysicalType_    Physical type of the vector.
 * \tparam PrimType_        Primitive type of the coordinates.
 * \tparam Dimension_       Dimension of the vector.
 * \ingroup vectors
 */
template<enum PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
class Vector : public VectorBase<Vector<PhysicalType_, PrimType_, Dimension_> >, private Eigen::Matrix<PrimType_, Dimension_, 1> {
 private:
  /*! \brief The base type.
   */
  typedef VectorBase<Vector<PhysicalType_, PrimType_, Dimension_> > Base;

  /*! \brief The size if the vector has dynamic dimension (must be equal to Eigen::Dynamic).
   */
  static constexpr int DynamicDimension = -1;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

  /*! \brief Default constructor for static sized vectors which initializes all components with zero.
   */
  template<int DimensionCopy_ = Dimension_>
  Vector(typename std::enable_if<DimensionCopy_ != DynamicDimension>::type* = nullptr)
    : Implementation(Implementation::Zero()) {
  }

  /*! \brief Default constructor for dynamic sized vectors.
   */
  template<int DimensionCopy_ = Dimension_>
  Vector(typename std::enable_if<DimensionCopy_ == DynamicDimension>::type* = nullptr)
    : Implementation() {
  }

  /*! \brief Constructor using other vector with generic type.
   *  \param other   Vector<OtherPhysicalType_, OtherPrimType_, Dimension_>
   */
  template<enum PhysicalType OtherPhysicalType_, typename OtherPrimType_>
  explicit Vector(const Vector<OtherPhysicalType_, OtherPrimType_, Dimension_>& other)
    : Implementation(other.toImplementation().template cast<PrimType_>()) {
  }

  /*! \brief Constructor of a dynamic vector using a static vector.
   *  \param other   Vector<OtherPhysicalType_, OtherPrimType_, Dimension_>
   */
  template<int DimensionOther_, int DimensionCopy_ = Dimension_>
  Vector(const Vector<PhysicalType_, PrimType_, DimensionOther_>& other, typename std::enable_if<DimensionCopy_ == DynamicDimension>::type* = nullptr)
    : Implementation(other.toImplementation()) {
  }

  /*! \brief Constructor using Eigen::Matrix.
   *  \param other   Eigen::Matrix<PrimType_,Dimension_,1>
   */
  explicit Vector(const Implementation& other)
    : Implementation(other) {
  }

  /*! \brief Constructor using single scalar. Only usable if Dimension_ is 1.
   * @param value
   */
  template <int DimensionCopy_ = Dimension_>
  explicit Vector(Scalar value, typename std::enable_if<DimensionCopy_ == 1>::type* = nullptr)
    : Implementation(value) {
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

  /*! \brief Get zero element.
   * \returns zero element
   */
  static Vector<PhysicalType_, PrimType_, Dimension_> Zero() {
    return Vector<PhysicalType_, PrimType_, Dimension_>(Implementation::Zero());
  }

  /*! \brief Sets all components of the vector to zero.
   * \returns reference
   */
  Vector<PhysicalType_, PrimType_, Dimension_>& setZero() {
    Implementation::setZero();
    return *this;
  }

  /*! \brief Get random element.
   * \returns random element
   */
  static Vector<PhysicalType_, PrimType_, Dimension_> Random() {
    return Vector<PhysicalType_, PrimType_, Dimension_>(Implementation::Random());
  }

  /*! \brief Sets all components of the vector to random.
   * \returns reference
   */
  Vector<PhysicalType_, PrimType_, Dimension_>& setRandom() {
    Implementation::setRandom();
    return *this;
  }

  /*! \brief Get the unity vector in x.
   * \returns the unity vector in x
   */
  static Vector<PhysicalType_, PrimType_, Dimension_> UnitX() {
    return Vector<PhysicalType_, PrimType_, Dimension_>(Implementation::UnitX());
  }

  /*! \brief Get the unity vector in y.
   * \returns the unity vector in y
   */
  static Vector<PhysicalType_, PrimType_, Dimension_> UnitY() {
    return Vector<PhysicalType_, PrimType_, Dimension_>(Implementation::UnitY());
  }

  /*! \brief Get the unity vector in z.
   * \returns the unity vector in z
   */
  static Vector<PhysicalType_, PrimType_, Dimension_> UnitZ() {
    return Vector<PhysicalType_, PrimType_, Dimension_>(Implementation::UnitZ());
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
  Vector<PhysicalType_, PrimType_, DimensionOutput_> getHead() const {
    return Vector<PhysicalType_, PrimType_, DimensionOutput_>(this->toImplementation().template head<DimensionOutput_>());
  }

  /*!\brief Get the head of the vector (copy)
   * \returns the head of the vector (copy)
   */
  Vector<PhysicalType_, PrimType_, DynamicDimension> getHead(int length) const {
    return Vector<PhysicalType_, PrimType_, DynamicDimension>(this->toImplementation().head(length));
  }

  /*!\brief Get the tail of the vector (copy)
   * \returns the tail of the vector (copy)
   */
  template<int DimensionOutput_>
  Vector<PhysicalType_, PrimType_, DimensionOutput_> getTail() const {
    return Vector<PhysicalType_, PrimType_, DimensionOutput_>(this->toImplementation().template tail<DimensionOutput_>());
  }

  /*!\brief Get the tail of the vector (copy)
   * \returns the tail of the vector (copy)
   */
  Vector<PhysicalType_, PrimType_, DynamicDimension> getTail(int length) const {
    return Vector<PhysicalType_, PrimType_, DynamicDimension>(this->toImplementation().tail(length));
  }

  /*!\brief Get a segment of the vector (copy)
   * \returns a segment of the vector (copy)
   */
  template<int DimensionOutput_>
  Vector<PhysicalType_, PrimType_, DimensionOutput_> getSegment(int start) const {
    return Vector<PhysicalType_, PrimType_, DimensionOutput_>(this->toImplementation().template segment<DimensionOutput_>(start));
  }

  /*!\brief Get a segment of the vector (copy)
   * \returns a segment of the vector (copy)
   */
  Vector<PhysicalType_, PrimType_, DynamicDimension> getSegment(int start, int length) const {
    return Vector<PhysicalType_, PrimType_, DynamicDimension>(this->toImplementation().segment(start, length));
  }

  /*!\brief Set the head of the vector
   */
  template<int DimensionInput_>
  void setHead(const Vector<PhysicalType_, PrimType_, DimensionInput_> & input) {
    this->toImplementation().template head<DimensionInput_>() = input.toImplementation();
  }

  /*!\brief Set the tail of the vector
   */
  template<int DimensionInput_>
  void setTail(const Vector<PhysicalType_, PrimType_, DimensionInput_> & input) {
    this->toImplementation().template tail<DimensionInput_>() = input.toImplementation();
  }

  /*!\brief Set a segment of the vector
   */
  template<int DimensionInput_>
  void setSegment(int start, const Vector<PhysicalType_, PrimType_, DimensionInput_> & input) {
    this->toImplementation().template segment<DimensionInput_>(start) = input.toImplementation();
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

  /*! \brief Cast to Eigen::Matrix<PrimType_, Dimension_, 1>.
   *  \returns Eigen::Matrix<PrimType_, Dimension_, 1>
   */
  inline Implementation& vector() {
    return static_cast<Implementation&>(*this);
  }

  /*! \brief Cast to Eigen::Matrix<PrimType_, Dimension_, 1>.
   *  \returns Eigen::Matrix<PrimType_, Dimension_, 1>
   */
  inline const Implementation& vector() const {
    return static_cast<const Implementation&>(*this);
  }

  /*! \brief Assignment operator.
   * \param other   other vector
   * \returns reference
   */
  // (The assignment of a static to a dynamic vector does not work because the amount of parameters must be one and SFINAE leads to two parameters. Workaround: cast the static vector into a dynamic one, then assign.)
  Vector<PhysicalType_, PrimType_, Dimension_> & operator=(const Vector<PhysicalType_, PrimType_, Dimension_>& other) = default;

  /*! \brief Addition of two vectors.
   * \param other   other vector
   * \returns sum
   */
  Vector<PhysicalType_, PrimType_, Dimension_> operator+(const Vector<PhysicalType_, PrimType_, Dimension_>& other) const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(this->toImplementation() + other.toImplementation());
  }

  /*! \brief Subtraction of two vectors.
   * \param other   other vector
   * \returns difference
   */
  Vector<PhysicalType_, PrimType_, Dimension_> operator-(const Vector<PhysicalType_, PrimType_, Dimension_>& other) const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(this->toImplementation() - other.toImplementation());
  }

  /*! \brief Multiplies vector with a scalar.
   * \param factor   factor
   * \returns product
   */
  template<typename PrimTypeFactor_, typename std::enable_if<std::is_arithmetic<PrimTypeFactor_>::value>::type* = nullptr>
  Vector<PhysicalType_, PrimType_, Dimension_> operator*(PrimTypeFactor_ factor) const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(this->toImplementation()*(PrimType_)factor);
  }

  /*! \brief Multiplies this vector with another vector. Note that at least one of the two vectors must have dimension 1.
   * \param factor   vector
   * \returns product
   */
  template <PhysicalType PhysicalTypeFactor_, typename PrimTypeFactor_, int DimensionFactor_>
  typename internal::MultiplicationReturnTypeTraits<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeFactor_, PrimTypeFactor_, DimensionFactor_>>::ReturnType
  operator*(const Vector<PhysicalTypeFactor_, PrimTypeFactor_, DimensionFactor_>& factor) const {
    return internal::MultiplicationTraits<
        Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeFactor_, PrimTypeFactor_, DimensionFactor_>>::multiply(*this, factor);
  }

  /*! \brief Divides vector by a scalar.
   * \param divisor   divisor
   * \returns quotient
   */
  template<typename PrimTypeDivisor_, typename std::enable_if<std::is_arithmetic<PrimTypeDivisor_>::value>::type* = nullptr>
  Vector<PhysicalType_, PrimType_, Dimension_> operator/(PrimTypeDivisor_ divisor) const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(this->toImplementation()/(PrimType_)divisor);
  }

  /*! \brief Divides vector by a scalar.
   * \param divisor   divisor with physical type
   * \returns quotient
   */
  template <PhysicalType PhysicalTypeDivisor_, typename PrimTypeDivisor_>
  typename internal::DivisionReturnTypeTraits<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeDivisor_, PrimTypeDivisor_, 1>>::ReturnType
  operator/(const Vector<PhysicalTypeDivisor_, PrimTypeDivisor_, 1>& divisor) const {
    return typename internal::DivisionReturnTypeTraits<
        Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeDivisor_, PrimTypeDivisor_, 1>>::ReturnType(
        this->toImplementation() / static_cast<PrimType_>(divisor.toImplementation()(0)));
  }

  /*! \brief Addition and assignment of two vectors.
   * \param other   other vector
   * \returns reference
   */
  Vector<PhysicalType_, PrimType_, Dimension_>& operator+=(const Vector<PhysicalType_, PrimType_, Dimension_>& other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  /*! \brief Subtraction and assignment of two vectors.
   * \param other   other vector
   * \returns reference
   */
  Vector<PhysicalType_, PrimType_, Dimension_>& operator-=(const Vector<PhysicalType_, PrimType_, Dimension_>& other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  /*! \brief Multiplication with a scalar and assignment.
   * \param factor   factor
   * \returns reference
   */
  template<typename PrimTypeFactor_>
  Vector<PhysicalType_, PrimType_, Dimension_>& operator*=(PrimTypeFactor_ factor) {
    this->toImplementation() *= (PrimType_)factor;
    return *this;
  }

  /*! \brief Multiplication with a scalar and assignment
   * @param factor  factor with physical type
   * @returns reference
   */
  template <typename PrimTypeFactor_>
  Vector<PhysicalType_, PrimType_, Dimension_>& operator*=(Vector<PhysicalType::Typeless, PrimTypeFactor_, 1> factor) {
    *this *= static_cast<PrimTypeFactor_>(factor.toImplementation()(0));
    return *this;
  }

  /*! \brief Division by a scalar and assignment.
   * \param divisor   divisor
   * \returns reference
   */
  template<typename PrimTypeDivisor_>
  Vector<PhysicalType_, PrimType_, Dimension_>& operator/=(PrimTypeDivisor_ divisor) {
    this->toImplementation() /= (PrimType_)divisor;
    return *this;
  }

  /*! \brief Division by a scalar and assignment
   * @param divisor  divisor with physical type
   * @returns reference
   */
  template <typename PrimTypeDivisor_>
  Vector<PhysicalType_, PrimType_, Dimension_>& operator/=(Vector<PhysicalType::Typeless, PrimTypeDivisor_, 1> divisor) {
    *this /= static_cast<PrimTypeDivisor_>(divisor.toImplementation()(0));
    return *this;
  }

  /*! \brief Negation of a vector.
   * \returns negative vector
   */
  Vector<PhysicalType_, PrimType_, Dimension_> operator-() const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(-this->toImplementation());
  }

  /*! \brief Comparison operator.
   * \param other   other vector
   * \returns true if equal
   */
  bool operator==(const Vector<PhysicalType_, PrimType_, Dimension_>& other) const {
    return this->toImplementation() == other.toImplementation();
  }

  /*! \brief Comparison operator.
   * \param other   other vector
   * \returns true if unequal
   */
  bool operator!=(const Vector<PhysicalType_, PrimType_, Dimension_>& other) const {
    return this->toImplementation() != other.toImplementation();
  }

  /*! \brief Comparison function.
   * \param other   other vector
   * \param tol   tolerance
   * \returns true if similar within tolerance
   */
  bool isSimilarTo(const Vector<PhysicalType_, PrimType_, Dimension_>& other, Scalar tol) const {
    if((*this - other).abs().max() < tol) {
      return true;
    } else {
      return false;
    }
  }

  /*! \brief Norm of the vector.
   *  \returns norm.
   */
  Scalar norm() const {
    return this->toImplementation().norm();
  }

  /*! \brief Squared norm of the vector.
   *  \returns norm.
   */
  Scalar squaredNorm() const {
    return this->toImplementation().squaredNorm();
  }

  /*! \brief Normalizes the vector.
   *  \returns reference.
   */
  Vector<PhysicalType_, PrimType_, Dimension_>& normalize() {
    this->toImplementation().normalize();
    return *this;
  }

  /*! \brief Get a normalized version of the vector.
   *  \returns normalized vector.
   */
  Vector<PhysicalType_, PrimType_, Dimension_> normalized() const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(this->toImplementation().normalized());
  }

  /*! \brief Dot product with other vector.
   *  \param other   other vector
   *  \returns dot product.
   */
  template<enum PhysicalType PhysicalTypeOther_>
  Scalar dot(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other) const {
    return this->toImplementation().dot(other.toImplementation());
  }

  /*! \brief Cross product with other vector.
   *  \param other   other vector
   *  \returns cross product.
   */
  template<enum PhysicalType PhysicalTypeOther_, int DimensionCopy_ = Dimension_>
  Vector<internal::MultiplicationPhysicalTypeTraits<PhysicalType_, PhysicalTypeOther_>::ReturnType, PrimType_, Dimension_>
  cross(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other, typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) const {
    return Vector<internal::MultiplicationPhysicalTypeTraits<PhysicalType_, PhysicalTypeOther_>::ReturnType, PrimType_, Dimension_>(this->toImplementation().cross(other.toImplementation()));
  }

  /*! \brief Projects this vector (a) on the other vector (b).
   *  The result  is
   *    proj = b/|b| * |a| * cos(angle) = b/|b| * |a| * a.b / |a| / |b|,
   *  which is computed by
   *           a.b
   *  proj = ------- * a
   *          |a|*|a|
   *  \param other   other vector
   *  \returns projected vector.
   */
  template<enum PhysicalType PhysicalTypeOther_>
  Vector<PhysicalType_, PrimType_, Dimension_> projectOn(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other) const {
    return other * (this->dot(other)/other.squaredNorm());
  }

  /*! \brief Elementwise product with other vector.
   *  \param other   other vector
   *  \returns elementwise product.
   */
  template<enum PhysicalType PhysicalTypeOther_>
  Vector<internal::MultiplicationPhysicalTypeTraits<PhysicalType_, PhysicalTypeOther_>::ReturnType, PrimType_, Dimension_>
  elementwiseMultiplication(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other) const {
    return Vector<internal::MultiplicationPhysicalTypeTraits<PhysicalType_, PhysicalTypeOther_>::ReturnType, PrimType_, Dimension_>(this->toImplementation().cwiseProduct(other.toImplementation()));
  }

  /*! \brief Elementwise division by other vector.
   *  \param other   other vector
   *  \returns elementwise quotient.
   */
  template<enum PhysicalType PhysicalTypeOther_>
  Vector<internal::DivisionPhysicalTypeTraits<PhysicalType_, PhysicalTypeOther_>::ReturnType, PrimType_, Dimension_>
  elementwiseDivision(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other) const {
    return Vector<internal::DivisionPhysicalTypeTraits<PhysicalType_, PhysicalTypeOther_>::ReturnType, PrimType_, Dimension_>(this->toImplementation().cwiseQuotient(other.toImplementation()));
  }

  /*! \brief Absolute components.
   *  \returns absolute components.
   */
  Vector<PhysicalType_, PrimType_, Dimension_> abs() const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(this->toImplementation().cwiseAbs());
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
  friend std::ostream& operator << (std::ostream& out, const Vector<PhysicalType_, PrimType_, Dimension_>& vector) {
    out << vector.transpose();
    return out;
  }
};

/*! \brief Multiplies a vector with a scalar.
 * \param factor   factor
 * \returns product
 */
template<enum PhysicalType PhysicalType_, typename PrimTypeFactor_, typename PrimType_, int Dimension_, typename std::enable_if<std::is_arithmetic<PrimTypeFactor_>::value>::type* = nullptr>
Vector<PhysicalType_, PrimType_, Dimension_> operator*(PrimTypeFactor_ factor, const Vector<PhysicalType_, PrimType_, Dimension_>& vector) {
  return vector*(PrimType_)factor;
}

namespace internal {

/*! \brief Gets the primitive type of the vector
 */
template<enum PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
class get_scalar<Vector<PhysicalType_, PrimType_, Dimension_>> {
 public:
  typedef PrimType_ Scalar;
};

/*! \brief Gets the dimension of the vector
 */
template<enum PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
class get_dimension<Vector<PhysicalType_, PrimType_, Dimension_>> {
 public:
  static constexpr int Dimension = Dimension_;
};

/*! \brief Trait to derive the dimension of a multiplication
 */
template<int Dimension1_, int Dimension2_, typename Enable_ = void>
class MultiplicationDimensionTraits
{
 public:
//  static constexpr int ReturnDimension = ...;

  // If this class is instantiated, there is no better fit for the trait, meaning that the dimensions mismatch.
  // Instead of throwing an error about inexistent `ReturnDimension`, the following static assert will always evaluate to `false`, producing a better error message.
  static_assert(Dimension1_ != Dimension1_, "Multiplication failed. Dimension mismatch.");
};

template<int Dimension_>
class MultiplicationDimensionTraits<Dimension_, 1>
{
 public:
  static constexpr int ReturnDimension = Dimension_;
};

template<int Dimension_>
class MultiplicationDimensionTraits<1, Dimension_, typename std::enable_if<Dimension_ != 1>::type>
{
 public:
  static constexpr int ReturnDimension = Dimension_;
};

/*! \brief Trait to derive the return type of a multiplication
 */
template<enum PhysicalType PhysicalType1_, enum PhysicalType PhysicalType2_, typename PrimType1_, typename PrimType2_, int Dimension1_, int Dimension2_>
class MultiplicationReturnTypeTraits<Vector<PhysicalType1_, PrimType1_, Dimension1_>, Vector<PhysicalType2_, PrimType2_, Dimension2_>>
{
 public:
  // Note: The returned primitive type is primitive type 1.
  typedef Vector<MultiplicationPhysicalTypeTraits<PhysicalType1_, PhysicalType2_>::ReturnType, PrimType1_, MultiplicationDimensionTraits<Dimension1_, Dimension2_>::ReturnDimension> ReturnType;
};

/*! \brief Trait to multiply two vectors, at least one of them having dimension 1
 */
template<enum PhysicalType PhysicalType1_, enum PhysicalType PhysicalType2_, typename PrimType1_, typename PrimType2_, int Dimension1_>
class MultiplicationTraits<Vector<PhysicalType1_, PrimType1_, Dimension1_>, Vector<PhysicalType2_, PrimType2_, 1>>
{
 public:
  typedef Vector<PhysicalType1_, PrimType1_, Dimension1_> Vector1;
  typedef Vector<PhysicalType2_, PrimType2_, 1> Vector2;

  static typename MultiplicationReturnTypeTraits<Vector1, Vector2>::ReturnType
  multiply(const Vector1& vector1, const Vector2& vector2) {
    return typename MultiplicationReturnTypeTraits<Vector1, Vector2>::ReturnType(
        vector1 * static_cast<PrimType1_>(vector2.toImplementation()(0)));
  }
};

template<enum PhysicalType PhysicalType1_, enum PhysicalType PhysicalType2_, typename PrimType1_, typename PrimType2_, int Dimension2_>
class MultiplicationTraits<Vector<PhysicalType1_, PrimType1_, 1>, Vector<PhysicalType2_, PrimType2_, Dimension2_>, typename std::enable_if<Dimension2_ != 1>::type>
{
 public:
  typedef Vector<PhysicalType1_, PrimType1_, 1> Vector1;
  typedef Vector<PhysicalType2_, PrimType2_, Dimension2_> Vector2;

  static typename MultiplicationReturnTypeTraits<Vector1, Vector2>::ReturnType
  multiply(const Vector1& vector1, const Vector2& vector2) {
    return MultiplicationTraits<Vector2, Vector1>::multiply(vector2, vector1);
  }
};

/*! \brief Trait to derive the dimension of a division
 */
template<int Dimension1_, int Dimension2_>
class DivisionDimensionTraits
{
 public:
//  static constexpr int ReturnDimension = ...;

  // If this class is instantiated, there is no better fit for the trait, meaning that the dimensions mismatch.
  // Instead of throwing an error about inexistent `ReturnDimension`, the following static assert will always evaluate to `false`, producing a better error message.
  static_assert(Dimension1_ != Dimension1_, "Division failed. Dimension mismatch.");
};

template<int Dimension_>
class DivisionDimensionTraits<Dimension_, 1>
{
 public:
  static constexpr int ReturnDimension = Dimension_;
};

/*! \brief Trait to derive the return type of a division
 */
template<enum PhysicalType PhysicalType1_, enum PhysicalType PhysicalType2_, typename PrimType_, int Dimension1_, int Dimension2_>
class DivisionReturnTypeTraits<Vector<PhysicalType1_, PrimType_, Dimension1_>, Vector<PhysicalType2_, PrimType_, Dimension2_>>
{
 public:
  typedef Vector<DivisionPhysicalTypeTraits<PhysicalType1_, PhysicalType2_>::ReturnType, PrimType_, DivisionDimensionTraits<Dimension1_, Dimension2_>::ReturnDimension> ReturnType;
};

/*! \brief Specializes multiplication and division traits for factor1 * factor2 -> product; and product / factor1 -> factor2
*/
#define KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_ONEWAY(FACTOR1, FACTOR2, PRODUCT) \
    template<> \
    class MultiplicationPhysicalTypeTraits<PhysicalType::FACTOR1, PhysicalType::FACTOR2> \
    { \
     public: \
      static constexpr PhysicalType ReturnType = PhysicalType::PRODUCT; \
    }; \
    template<> \
    class DivisionPhysicalTypeTraits<PhysicalType::PRODUCT, PhysicalType::FACTOR2> \
    { \
     public: \
      static constexpr PhysicalType ReturnType = PhysicalType::FACTOR1; \
    };

/*! \brief Specializes multiplication and division traits for the triple (factor1 != factor2)
*/
#define KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(FACTOR1, FACTOR2, PRODUCT) \
    KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_ONEWAY(FACTOR1, FACTOR2, PRODUCT) \
    KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_ONEWAY(FACTOR2, FACTOR1, PRODUCT)

/*! \brief Specializes multiplication and division traits for the triple (factor1 == factor2)
 */
#define KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_B(FACTOR1AND2, PRODUCT) \
    KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_ONEWAY(FACTOR1AND2, FACTOR1AND2, PRODUCT)


KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_B(Typeless, Typeless)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Time, Time)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Jerk, Jerk)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Acceleration, Acceleration)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Velocity, Velocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Position, Position) // Position/Position = Typeless
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Force, Force)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Momentum, Momentum)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, AngularJerk, AngularJerk)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, AngularAcceleration, AngularAcceleration)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, AngularVelocity, AngularVelocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Angle, Angle)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, Torque, Torque)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Typeless, AngularMomentum, AngularMomentum)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Position, AngularJerk, Jerk)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Position, AngularAcceleration, Acceleration)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Position, AngularVelocity, Velocity)
//KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Position, Angle, Position) // Position/Position = Angle -> ambiguous, explicit cast to Angle if needed
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Position, Force, Torque)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Position, Momentum, AngularMomentum)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Jerk, Acceleration)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Acceleration, Velocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Velocity, Position)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Force, Momentum)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, AngularJerk, AngularAcceleration)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, AngularAcceleration, AngularVelocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, AngularVelocity, Angle)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Torque, AngularMomentum)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Mass, Acceleration, Force)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Mass, Velocity, Momentum)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Inertia, AngularAcceleration, Torque)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Inertia, AngularVelocity, AngularMomentum)



} // namespace internal
} // namespace kindr




