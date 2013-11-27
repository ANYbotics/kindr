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

#ifndef KINDER_POSITIONEIGEN_HPP_
#define KINDER_POSITIONEIGEN_HPP_

#include "kinder/common/common.hpp"
#include "kinder/common/assert_macros_eigen.hpp"
#include "PositionBase.hpp"

namespace kinder {
namespace positions {
//! Implementation of rotations based on the C++ Eigen library
namespace eigen_implementation {

template<typename PrimType>
class Position3 : public Position3Base<Position3<PrimType>>, private Eigen::Matrix<PrimType, 3, 1> {
 private:
  /*! \brief The base type.
   */
  typedef Eigen::Matrix<PrimType, 3, 1> Base;
 public:
  /*! \brief The implementation type.
   *  The implementation type is always an Eigen object.
   */
  typedef Base Implementation;

  /*! \brief The primary type.
   *  Float/Double
   */
  typedef PrimType Scalar;

  /*! \brief Default constructor initializes all coefficients with zero.
   */
  Position3()
    : Base(Base::Zero()) {
  }

  Position3(const PrimType & x, const PrimType & y, const PrimType & z)
    : Base(x, y, z) {
  }


  /*! \brief Constructor using Eigen::Vector3.
   *  \param other   Eigen::Matrix<PrimType,3,1>
   */
  explicit Position3(const Base & other)
    : Base(other) {
   }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline Implementation & toImplementation() {
    return static_cast<Implementation &>(*this);
  }

  /*! \brief Cast to the implementation type.
   *  \returns the implementation (recommended only for advanced users)
   */
  inline const Implementation & toImplementation() const {
    return static_cast<const Implementation &>(*this);
  }

  using Position3Base<Position3<PrimType>>::operator+; // otherwise ambiguous PositionBase and Eigen
  using Position3Base<Position3<PrimType>>::operator-; // otherwise ambiguous PositionBase and Eigen


  template<typename OTHER>
  Position3<PrimType> & operator +=(const OTHER & other) {
    this->toImplementation() += other.toImplementation();
    return *this;
  }

  template<typename OTHER>
  Position3<PrimType> & operator -=(const OTHER & other) {
    this->toImplementation() -= other.toImplementation();
    return *this;
  }

  Position3 & setZero() {
    Base::setZero();
  }

  inline const Scalar& x() const {
    return Base::x();
  }

  inline Scalar& x() {
    return Base::x();
  }

  inline const Scalar& y() const {
    return Base::y();
  }

  inline Scalar& y() {
    return Base::y();
  }

  inline const Scalar& z() const {
    return Base::z();
  }

  inline Scalar& z() {
    return Base::z();
  }

  /*! \brief Used for printing the object with std::cout.
   *  \returns std::stream object
   */
  friend std::ostream & operator << (std::ostream & out, const Position3 & position) {
    out << position.transpose();
    return out;
  }
};


//! \brief 3D-Position with double primary type
typedef Position3<double>  Position3D;

//! \brief 3D-Position with float primary type
typedef Position3<float>  Position3F;

} // namespace eigen_implementation

namespace internal {

template<typename PrimType>
class get_scalar<eigen_implementation::Position3<PrimType>>{
 public:
  typedef PrimType Scalar;
};


//template<typename PrimType>
//class AdditionTraits<PositionBase<eigen_implementation::Position3<PrimType>>, PositionBase<eigen_implementation::Position3<PrimType>>> {
// public:
//  inline static eigen_implementation::Position3<PrimType> add(const eigen_implementation::Position3<PrimType> & a, const eigen_implementation::Position3<PrimType> & b) {
//    return eigen_implementation::Position3<PrimType(
//                                                                   a.toImplementation()*
//                                                                   b.toImplementation());
//  }
//};

} // namespace internal
} // namespace positions
} // namespace kinder





#endif /* KINDER_POSITIONEIGEN_HPP_ */
