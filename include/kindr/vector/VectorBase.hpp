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

#ifndef KINDR_PHYSICALQUANTITIES_VECTORBASE_HPP_
#define KINDR_PHYSICALQUANTITIES_VECTORBASE_HPP_


#include "kindr/common/common.hpp"

namespace kindr {
//! Generic vector interface
/*! \ingroup vectors
 */
namespace vector {
//! Internal stuff (only for developers)
namespace internal {

/*! \brief Addition traits for vectors
 *  \class AdditionTraits
 *  (only for advanced users)
 */
template<typename LeftAndRight_>
class AdditionTraits {
 public:
//  inline static LeftAndRight_ add(const LeftAndRight_& lhs, const LeftAndRight_& rhs);
//  inline static LeftAndRight_ subtract(const LeftAndRight_& lhs, const LeftAndRight_& rhs);
};

/*! \brief Addition traits for vectors
 *  \class AdditionTraits
 *  (only for advanced users)
 */
template<typename Derived_>
class AccessTraits {
 public:
//  inline static Derived_ access(const Derived_& vector, int index);
//  inline static Derived_& accessRef(const Derived_& vector, int index);
};

/*! \class get_scalar
 *  \brief Gets the primitive of the vector.
 */
template<typename Vector_>
class get_scalar {
 public:
//  typedef PrimType Scalar;
};

/*! \class get_dimension
 *  \brief Gets the dimension of the vector.
 */
template<typename Vector_>
class get_dimension {
 public:
//  typedef PrimType Scalar;
};

} // namespace internal

/*! \class VectorBase
 * \brief Interface for a vector.
 *
 * This class defines the generic interface for a vector.
 * More precisely an interface to store and access the coordinates of a vector of a point is provided.
 * \tparam Derived_ the derived class that should implement the vector.
 * \ingroup vectors
 *
 */
template<typename Derived_>
class VectorBase {
 public:
  /*! \brief The primitive type of a vector coordinate.
   */
  typedef typename internal::get_scalar<Derived_>::Scalar Scalar;

  /*! \brief Default constructor.
   *
    *  Creates a vector with zero coefficients.
    */
  VectorBase() = default;

  /*! \brief Constructor from derived vector.
   *
   *  This constructor has been deleted because the abstract class does not contain any data.
   */
  VectorBase(const Derived_&) = delete; // on purpose!!

  /*! \brief Gets the derived vector.
   *  (only for advanced users)
   *  \returns the derived vector
   */
  operator Derived_& () {
    return static_cast<Derived_&>(*this);
  }

  /*! \brief Gets the derived vector.
   *  (only for advanced users)
   *  \returns the derived vector
   */
  operator const Derived_& () const {
    return static_cast<const Derived_&>(*this);
  }

  /*! \brief Gets the derived vector.
   *  (only for advanced users)
   *  \returns the derived vector
   */
  const Derived_& derived() const {
    return static_cast<const Derived_&>(*this);
  }


  /*! \brief Sets the vector to zero.
   *  \returns reference
   */
  Derived_& setZero();

  /*! \brief Accesses an entry of the vector.
   *  \returns A copy of an entry of the vector.
   */
  Scalar operator ()(int index) const {
    return internal::AdditionTraits<VectorBase<Derived_>>::acces(this->derived(), index);
  }

  /*! \brief Accesses an entry of the vector.
   *  \returns A reference to an entry of the vector.
   */
  Scalar& operator ()(int index) {
    return internal::AdditionTraits<VectorBase<Derived_>>::accessRef(this->derived(), index);
  }

  /*! \brief Addition of two vectors.
   *  \returns sum of the two vectors.
   */
  template<typename OtherDerived_>
  Derived_ operator +(const VectorBase<OtherDerived_>& other) const {
    return internal::AdditionTraits<VectorBase<Derived_>>::add(this->derived(), other.derived());
  }

  /*! \brief Subtraction of two vectors.
   *  \returns result of the subtraction of the two vectors.
   */
  template<typename OtherDerived_>
  Derived_ operator -(const VectorBase<OtherDerived_>& other) const {
    return internal::AdditionTraits<VectorBase<Derived_>>::subtract(this->derived(), other.derived());
  }

  /*! \brief Addition and assignment.
   *  \returns sum of the two vectors.
   */
  template<typename OtherDerived_>
  Derived_& operator +=(const VectorBase<OtherDerived_>& other);

  /*! \brief Subtraction and assignment.
   *  \returns result of the subtraction of the two vectors.
   */
  template<typename OtherDerived_>
  Derived_& operator -=(const VectorBase<OtherDerived_>& other);

  /*! \brief Norm of the vector.
   *  \returns norm.
   */
  Scalar norm() const;

  /*! \brief Normalizes the vector.
   *  \returns reference.
   */
  Derived_& normalize();

  /*! \brief Get a normalized version of the vector.
   *  \returns normalized vector.
   */
  Derived_ normalized() const;

  /*! \brief Dot product with other vector.
   *  \returns dot product.
   */
  Scalar dot(const VectorBase<Derived_>& other) const;

  /*! \brief Cross product with other vector.
   *  \returns cross product.
   */
  Derived_ cross(const VectorBase<Derived_>& other) const;
};


namespace internal {

template<typename LeftAndRight_>
class AdditionTraits<VectorBase<LeftAndRight_>> {
 public:
  /*! \brief The primitive type of a vector coordinate.
   */
  typedef typename internal::get_scalar<LeftAndRight_>::Scalar Scalar;
  /*! \returns the sum of two vectors
   * \param lhs left-hand side
   * \param rhs right-hand side
   */
  inline static LeftAndRight_ add(const VectorBase<LeftAndRight_>& lhs, const VectorBase<LeftAndRight_>& rhs) {
    return LeftAndRight_(typename LeftAndRight_::Implementation(lhs.derived().toImplementation() + rhs.derived().toImplementation()));
  }
  /*! \returns the subtraction of two vectors
   * \param lhs left-hand side
   * \param rhs right-hand side
   */
  inline static LeftAndRight_ subtract(const VectorBase<LeftAndRight_>& lhs, const VectorBase<LeftAndRight_>& rhs) {
    return LeftAndRight_(typename LeftAndRight_::Implementation(lhs.derived().toImplementation() - rhs.derived().toImplementation()));
  }
};

template<typename Derived_>
class AccessTraits<VectorBase<Derived_>> {
 public:
  /*! \brief The primitive type of a vector coordinate.
   */
  typedef typename internal::get_scalar<Derived_>::Scalar Scalar;
  /*! \returns A copy of an entry of the vector.
   * \param vector Vector
   * \param index Index
   */
  inline static Scalar access(const VectorBase<Derived_>& vector, int index) {
    return vector.derived().toImplementation()(index);
  }
  /*! \returns A copy of an entry of the vector.
   * \param vector Vector
   * \param index Index
   */
  inline static Scalar& accessRef(const VectorBase<Derived_>& vector, int index) {
    return vector.derived().toImplementation()(index);
  }
};

} // namespace internal

} // namespace vector
} // namespace kindr


#endif /* KINDR_PHYSICALQUANTITIES_VECTORBASE_HPP_ */
