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
#include "kindr/phys_quant/PhysicalType.hpp"

namespace kindr {
namespace vector {
//! Implementation of rotations based on the C++ Eigen library
namespace eigen_impl {



/*! \class Vector
 * \brief Vector in n-dimensional-space.
 *
 * This class implements a vector in n-dimensional-space.
 * More precisely an interface to store and access the coordinates of a vector of a point in n-dimensional-space is provided.
 * \tparam PhysicalType_  Physical type of the vector.
 * \tparam PrimType_  Primitive type of the coordinates.
 * \tparam Dimension_  Dimension of the vector.
 * \ingroup vectors
 */
template<enum phys_quant::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
class Vector : public VectorBase<Vector<PhysicalType_, PrimType_, Dimension_> >, private Eigen::Matrix<PrimType_, Dimension_, 1> {
 private:
  /*! \brief The base type.
   */
  typedef VectorBase<Vector<PhysicalType_, PrimType_, Dimension_> > Base;
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

  /*! \brief Constructor using other vector without physical type.
   *  \param other   Vector<OtherPhysicalType_, OtherPrimType_, Dimension_>
   */
  template<enum phys_quant::PhysicalType OtherPhysicalType_, typename OtherPrimType_>
  explicit Vector(const Vector<OtherPhysicalType_, OtherPrimType_, Dimension_>& other)
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
  Vector<PhysicalType_, PrimType_, DimensionOutput_> head() const {
    return Vector<PhysicalType_, PrimType_, DimensionOutput_>(this->toImplementation().head(DimensionOutput_));
  }

  /*!\brief Get the tail of the vector (copy)
   * \returns the tail of the vector (copy)
   */
  template<int DimensionOutput_>
  Vector<PhysicalType_, PrimType_, DimensionOutput_> tail() const {
    return Vector<PhysicalType_, PrimType_, DimensionOutput_>(this->toImplementation().tail(DimensionOutput_));
  }

  /*!\brief Get a segment of the vector (copy)
   * \returns a segment of the vector (copy)
   */
  template<int Start_, int DimensionOutput_>
  Vector<PhysicalType_, PrimType_, DimensionOutput_> segment() const {
    return Vector<PhysicalType_, PrimType_, DimensionOutput_>(this->toImplementation().block(Start_,0,DimensionOutput_,1)); // todo: use templated block()
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
  Scalar norm() {
    return this->toImplementation().norm();
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
  template<enum phys_quant::PhysicalType PhysicalTypeOther_>
  Scalar dot(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other) const {
    return this->toImplementation().dot(other.toImplementation());
  }

  /*! \brief Cross product with other vector.
   *  \param other   other vector
   *  \returns cross product.
   */
  template<enum phys_quant::PhysicalType PhysicalTypeOther_, int DimensionCopy_ = Dimension_>
  typename internal::MultiplicationReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType
  cross(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other, typename std::enable_if<DimensionCopy_ == 3>::type* = nullptr) const {
    return typename internal::MultiplicationReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType(this->toImplementation().cross(other.toImplementation()));
  }

  /*! \brief Elementwise product with other vector.
   *  \param other   other vector
   *  \returns elementwise product.
   */
  template<enum phys_quant::PhysicalType PhysicalTypeOther_>
  typename internal::MultiplicationReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType
  elementwiseMultiplication(const Vector<PhysicalTypeOther_, PrimType_, Dimension_>& other) const {
    return typename internal::MultiplicationReturnTypeTrait<Vector<PhysicalType_, PrimType_, Dimension_>, Vector<PhysicalTypeOther_, PrimType_, Dimension_>>::ReturnType(this->toImplementation().cwiseProduct(other.toImplementation()));
  }

  /*! \brief Elementwise division by other vector.
   *  \param other   other vector
   *  \returns elementwise quotient.
   */
  template<enum phys_quant::PhysicalType PhysicalTypeOther_>
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
template<enum phys_quant::PhysicalType PhysicalType_, typename PrimTypeFactor_, typename PrimType_, int Dimension_>
Vector<PhysicalType_, PrimType_, Dimension_> operator*(PrimTypeFactor_ factor, const Vector<PhysicalType_, PrimType_, Dimension_>& vector) {
  return vector*(PrimType_)factor;
}

//! \brief Unitless-Vector
template <typename PrimType_, int Dimension_>
using VectorUnitless = Vector<phys_quant::PhysicalType::Undefined, PrimType_, Dimension_>;
//! \brief 3D-Unitless-Vector with primitive type double
typedef VectorUnitless<double, 3> Vector3D;
//! \brief 3D-Unitless-Vector with primitive type float
typedef VectorUnitless<float,  3> Vector3F;



//! \brief Length-Vector
template <typename PrimType_, int Dimension_>
using Length = Vector<phys_quant::PhysicalType::Length, PrimType_, Dimension_>;
//! \brief 3D-Length-Vector with primitive type double
typedef Length<double, 3> Length3D;
//! \brief 3D-Length-Vector with primitive type float
typedef Length<float,  3> Length3F;

//! \brief Velocity-Vector
template <typename PrimType_, int Dimension_>
using Velocity = Vector<phys_quant::PhysicalType::Velocity, PrimType_, Dimension_>;
//! \brief 3D-Velocity-Vector with primitive type double
typedef Velocity<double, 3> Velocity3D;
//! \brief 3D-Velocity-Vector with primitive type float
typedef Velocity<float,  3> Velocity3F;

//! \brief Acceleration-Vector
template <typename PrimType_, int Dimension_>
using Acceleration = Vector<phys_quant::PhysicalType::Acceleration, PrimType_, Dimension_>;
//! \brief 3D-Acceleration-Vector with primitive type double
typedef Acceleration<double, 3> Acceleration3D;
//! \brief 3D-Acceleration-Vector with primitive type float
typedef Acceleration<float,  3> Acceleration3F;

//! \brief Force-Vector
template <typename PrimType_, int Dimension_>
using Force = Vector<phys_quant::PhysicalType::Force, PrimType_, Dimension_>;
//! \brief 3D-Force-Vector with primitive type double
typedef Force<double, 3> Force3D;
//! \brief 3D-Force-Vector with primitive type float
typedef Force<float,  3> Force3F;

//! \brief Momentum-Vector
template <typename PrimType_, int Dimension_>
using Momentum = Vector<phys_quant::PhysicalType::Momentum, PrimType_, Dimension_>;
//! \brief 3D-Momentum-Vector with primitive type double
typedef Momentum<double, 3> Momentum3D;
//! \brief 3D-Momentum-Vector with primitive type float
typedef Momentum<float,  3> Momentum3F;



//! \brief Angle-Vector
template <typename PrimType_, int Dimension_>
using Angle = Vector<phys_quant::PhysicalType::Angle, PrimType_, Dimension_>;
//! \brief 3D-Angle-Vector with primitive type double
typedef Angle<double, 3> Angle3D;
//! \brief 3D-Angle-Vector with primitive type float
typedef Angle<float,  3> Angle3F;

//! \brief AngularVelocity-Vector
template <typename PrimType_, int Dimension_>
using AngularVelocity = Vector<phys_quant::PhysicalType::AngularVelocity, PrimType_, Dimension_>;
//! \brief 3D-AngularVelocity-Vector with primitive type double
typedef AngularVelocity<double, 3> AngularVelocity3D;
//! \brief 3D-AngularVelocity-Vector with primitive type float
typedef AngularVelocity<float,  3> AngularVelocity3F;

//! \brief AngularAcceleration-Vector
template <typename PrimType_, int Dimension_>
using AngularAcceleration = Vector<phys_quant::PhysicalType::AngularAcceleration, PrimType_, Dimension_>;
//! \brief 3D-AngularAcceleration-Vector with primitive type double
typedef AngularAcceleration<double, 3> AngularAcceleration3D;
//! \brief 3D-AngularAcceleration-Vector with primitive type float
typedef AngularAcceleration<float,  3> AngularAcceleration3F;

//! \brief Torque-Vector
template <typename PrimType_, int Dimension_>
using Torque = Vector<phys_quant::PhysicalType::Torque, PrimType_, Dimension_>;
//! \brief 3D-Torque-Vector with primitive type double
typedef Torque<double, 3> Torque3D;
//! \brief 3D-Torque-Vector with primitive type float
typedef Torque<float,  3> Torque3F;

//! \brief AngularMomentum-Vector
template <typename PrimType_, int Dimension_>
using AngularMomentum = Vector<phys_quant::PhysicalType::AngularMomentum, PrimType_, Dimension_>;
//! \brief 3D-AngularMomentum-Vector with primitive type double
typedef AngularMomentum<double, 3> AngularMomentum3D;
//! \brief 3D-AngularMomentum-Vector with primitive type float
typedef AngularMomentum<float,  3> AngularMomentum3F;



} // namespace eigen_impl



namespace internal {

/*! \brief Gets the primitive type of the vector
 */
template<enum phys_quant::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
class get_scalar<eigen_impl::Vector<PhysicalType_, PrimType_, Dimension_>> {
 public:
  typedef PrimType_ Scalar;
};

/*! \brief Gets the dimension of the vector
 */
template<enum phys_quant::PhysicalType PhysicalType_, typename PrimType_, int Dimension_>
class get_dimension<eigen_impl::Vector<PhysicalType_, PrimType_, Dimension_>> {
 public:
  static constexpr int Dimension = Dimension_;
};

/*! \brief Gets the return type of a multiplication
 */
template<enum phys_quant::PhysicalType PhysicalType1_, enum phys_quant::PhysicalType PhysicalType2_, typename PrimType_, int Dimension_>
class MultiplicationReturnTypeTrait<eigen_impl::Vector<PhysicalType1_, PrimType_, Dimension_>, eigen_impl::Vector<PhysicalType2_, PrimType_, Dimension_>>
{
 public:
  typedef eigen_impl::Vector<phys_quant::PhysicalType::Undefined, PrimType_, Dimension_> ReturnType;
};

/*! \brief Gets the return type of a multiplication
 */
template<enum phys_quant::PhysicalType PhysicalType1_, enum phys_quant::PhysicalType PhysicalType2_, typename PrimType_, int Dimension_>
class DivisionReturnTypeTrait<eigen_impl::Vector<PhysicalType1_, PrimType_, Dimension_>, eigen_impl::Vector<PhysicalType2_, PrimType_, Dimension_>>
{
 public:
  typedef eigen_impl::Vector<phys_quant::PhysicalType::Undefined, PrimType_, Dimension_> ReturnType;
};

/*! \brief Specializes multiplication and division traits for the triple (factor1 != factor2)
 */
#define KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(FACTOR1, FACTOR2, PRODUCT) \
    template<typename PrimType_, int Dimension_> \
    class MultiplicationReturnTypeTrait<eigen_impl::Vector<phys_quant::PhysicalType::FACTOR1, PrimType_, Dimension_>, eigen_impl::Vector<phys_quant::PhysicalType::FACTOR2, PrimType_, Dimension_>> \
    { \
     public: \
      typedef eigen_impl::Vector<phys_quant::PhysicalType::PRODUCT, PrimType_, Dimension_> ReturnType; \
    }; \
    template<typename PrimType_, int Dimension_> \
    class MultiplicationReturnTypeTrait<eigen_impl::Vector<phys_quant::PhysicalType::FACTOR2, PrimType_, Dimension_>, eigen_impl::Vector<phys_quant::PhysicalType::FACTOR1, PrimType_, Dimension_>> \
    { \
     public: \
      typedef eigen_impl::Vector<phys_quant::PhysicalType::PRODUCT, PrimType_, Dimension_> ReturnType; \
    }; \
    template<typename PrimType_, int Dimension_> \
    class DivisionReturnTypeTrait<eigen_impl::Vector<phys_quant::PhysicalType::PRODUCT, PrimType_, Dimension_>, eigen_impl::Vector<phys_quant::PhysicalType::FACTOR1, PrimType_, Dimension_>> \
    { \
     public: \
      typedef eigen_impl::Vector<phys_quant::PhysicalType::FACTOR2, PrimType_, Dimension_> ReturnType; \
    }; \
    template<typename PrimType_, int Dimension_> \
    class DivisionReturnTypeTrait<eigen_impl::Vector<phys_quant::PhysicalType::PRODUCT, PrimType_, Dimension_>, eigen_impl::Vector<phys_quant::PhysicalType::FACTOR2, PrimType_, Dimension_>> \
    { \
     public: \
      typedef eigen_impl::Vector<phys_quant::PhysicalType::FACTOR1, PrimType_, Dimension_> ReturnType; \
    };

/*! \brief Specializes multiplication and division traits for the triple (factor1 == factor2)
 */
#define KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_B(FACTOR1AND2, PRODUCT) \
    template<typename PrimType_, int Dimension_> \
    class MultiplicationReturnTypeTrait<eigen_impl::Vector<phys_quant::PhysicalType::FACTOR1AND2, PrimType_, Dimension_>, eigen_impl::Vector<phys_quant::PhysicalType::FACTOR1AND2, PrimType_, Dimension_>> \
    { \
     public: \
      typedef eigen_impl::Vector<phys_quant::PhysicalType::PRODUCT, PrimType_, Dimension_> ReturnType; \
    }; \
    template<typename PrimType_, int Dimension_> \
    class DivisionReturnTypeTrait<eigen_impl::Vector<phys_quant::PhysicalType::PRODUCT, PrimType_, Dimension_>, eigen_impl::Vector<phys_quant::PhysicalType::FACTOR1AND2, PrimType_, Dimension_>> \
    { \
     public: \
      typedef eigen_impl::Vector<phys_quant::PhysicalType::FACTOR1AND2, PrimType_, Dimension_> ReturnType; \
    };

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_B(Undefined, Undefined)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Time, Time)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Mass, Mass)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Inertia, Inertia)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Energy, Energy)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Length, Length) // Length/Length = Undefined
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Velocity, Velocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Acceleration, Acceleration)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Jerk, Jerk)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Force, Force)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Momentum, Momentum)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Angle, Angle)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, AngularVelocity, AngularVelocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, AngularAcceleration, AngularAcceleration)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, AngularJerk, AngularJerk)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, Torque, Torque)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Undefined, AngularMomentum, AngularMomentum)

//KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Length, Angle, Length) // Length/Length = Angle -> ambiguous, explicit cast to Angle if needed
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Length, AngularVelocity, Velocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Length, AngularAcceleration, Acceleration)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Length, AngularJerk, Jerk)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Length, Force, Torque)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Length, Momentum, AngularMomentum)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Acceleration, Velocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Velocity, Length)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, AngularAcceleration, AngularVelocity)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, AngularVelocity, Angle)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Jerk, Force)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Force, Momentum)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, AngularJerk, Torque)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Time, Torque, AngularMomentum)

KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Mass, Acceleration, Force)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Mass, Velocity, Momentum)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Inertia, AngularAcceleration, Torque)
KINDR_SPECIALIZE_PHYS_QUANT_RETURN_TYPE_A(Inertia, AngularVelocity, AngularMomentum)



} // namespace internal


} // namespace vector
} // namespace kindr





#endif /* KINDR_PHYSICALQUANTITIES_VECTOR_HPP_ */
