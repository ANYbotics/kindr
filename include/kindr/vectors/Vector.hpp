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
  Vector<PhysicalType_, PrimType_, Dimension_> & operator=(const Vector<PhysicalType_, PrimType_, Dimension_>& other) { // (The assignment of a static to a dynamic vector does not work because the amount of parameters must be one and SFINAE leads to two parameters. Workaround: cast the static vector into a dynamic one, then assign.)
    this->toImplementation() = other.toImplementation();
    return *this;
  }

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
  template<typename PrimTypeFactor_>
  Vector<PhysicalType_, PrimType_, Dimension_> operator*(PrimTypeFactor_ factor) const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(this->toImplementation()*(PrimType_)factor);
  }

  /*! \brief Divides vector by a scalar.
   * \param divisor   divisor
   * \returns quotient
   */
  template<typename PrimTypeDivisor_>
  Vector<PhysicalType_, PrimType_, Dimension_> operator/(PrimTypeDivisor_ divisor) const {
    return Vector<PhysicalType_, PrimType_, Dimension_>(this->toImplementation()/(PrimType_)divisor);
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

  /*! \brief Division by a scalar and assignment.
   * \param divisor   divisor
   * \returns reference
   */
  template<typename PrimTypeDivisor_>
  Vector<PhysicalType_, PrimType_, Dimension_>& operator/=(PrimTypeDivisor_ divisor) {
    this->toImplementation() /= (PrimType_)divisor;
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
  typename internal::MultiplicationReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType
  cross(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other, typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) const {
    return typename internal::MultiplicationReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType(this->toImplementation().cross(other.toImplementation()));
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
  typename internal::MultiplicationReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType
  elementwiseMultiplication(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other) const {
    return typename internal::MultiplicationReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType(this->toImplementation().cwiseProduct(other.toImplementation()));
  }

  /*! \brief Elementwise division by other vector.
   *  \param other   other vector
   *  \returns elementwise quotient.
   */
  template<enum PhysicalType PhysicalTypeOther_>
  typename internal::DivisionReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType
  elementwiseDivision(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other) const {
    return typename internal::DivisionReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType(this->toImplementation().cwiseQuotient(other.toImplementation()));
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
template<enum PhysicalType PhysicalType_, typename PrimTypeFactor_, typename PrimType_, int Dimension_>
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

/*! \brief Gets the return type of a multiplication
 */
template<enum PhysicalType PhysicalType1_, enum PhysicalType PhysicalType2_, typename PrimType_, int Dimension_>
class MultiplicationReturnTypeTrait<Vector<PhysicalType1_, PrimType_, Dimension_>, Vector<PhysicalType2_, PrimType_, Dimension_>>
{
 public:
  typedef Vector<PhysicalType::Typeless, PrimType_, Dimension_> ReturnType;
};

/*! \brief Gets the return type of a multiplication
 */
template<enum PhysicalType PhysicalType1_, enum PhysicalType PhysicalType2_, typename PrimType_, int Dimension_>
class DivisionReturnTypeTrait<Vector<PhysicalType1_, PrimType_, Dimension_>, Vector<PhysicalType2_, PrimType_, Dimension_>>
{
 public:
  typedef Vector<PhysicalType::Typeless, PrimType_, Dimension_> ReturnType;
};

/*! \brief Specializes multiplication and division traits for the triple (factor1 != factor2)
 */
#define KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(FACTOR1, FACTOR2, PRODUCT) \
    template<typename PrimType_, int Dimension_> \
    class MultiplicationReturnTypeTrait<Vector<PhysicalType::FACTOR1, PrimType_, Dimension_>, Vector<PhysicalType::FACTOR2, PrimType_, Dimension_>> \
    { \
     public: \
      typedef Vector<PhysicalType::PRODUCT, PrimType_, Dimension_> ReturnType; \
    }; \
    template<typename PrimType_, int Dimension_> \
    class MultiplicationReturnTypeTrait<Vector<PhysicalType::FACTOR2, PrimType_, Dimension_>, Vector<PhysicalType::FACTOR1, PrimType_, Dimension_>> \
    { \
     public: \
      typedef Vector<PhysicalType::PRODUCT, PrimType_, Dimension_> ReturnType; \
    }; \
    template<typename PrimType_, int Dimension_> \
    class DivisionReturnTypeTrait<Vector<PhysicalType::PRODUCT, PrimType_, Dimension_>, Vector<PhysicalType::FACTOR1, PrimType_, Dimension_>> \
    { \
     public: \
      typedef Vector<PhysicalType::FACTOR2, PrimType_, Dimension_> ReturnType; \
    }; \
    template<typename PrimType_, int Dimension_> \
    class DivisionReturnTypeTrait<Vector<PhysicalType::PRODUCT, PrimType_, Dimension_>, Vector<PhysicalType::FACTOR2, PrimType_, Dimension_>> \
    { \
     public: \
      typedef Vector<PhysicalType::FACTOR1, PrimType_, Dimension_> ReturnType; \
    };

/*! \brief Specializes multiplication and division traits for the triple (factor1 == factor2)
 */
#define KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_B(FACTOR1AND2, PRODUCT) \
    template<typename PrimType_, int Dimension_> \
    class MultiplicationReturnTypeTrait<Vector<PhysicalType::FACTOR1AND2, PrimType_, Dimension_>, Vector<PhysicalType::FACTOR1AND2, PrimType_, Dimension_>> \
    { \
     public: \
      typedef Vector<PhysicalType::PRODUCT, PrimType_, Dimension_> ReturnType; \
    }; \
    template<typename PrimType_, int Dimension_> \
    class DivisionReturnTypeTrait<Vector<PhysicalType::PRODUCT, PrimType_, Dimension_>, Vector<PhysicalType::FACTOR1AND2, PrimType_, Dimension_>> \
    { \
     public: \
      typedef Vector<PhysicalType::FACTOR1AND2, PrimType_, Dimension_> ReturnType; \
    };

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




