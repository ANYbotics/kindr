/*
 * Force.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_FORCE_HPP_
#define KINDR_PHYS_QUANT_FORCE_HPP_


#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {


template<typename PrimType_, int Dimension_>
class Force : public vector::eigen_impl::Vector<PrimType_, Dimension_>
{
 private:
  typedef vector::eigen_impl::Vector<PrimType_, Dimension_> Base;

 public:
  typedef typename Base::Implementation Implementation;
  typedef PrimType_ Scalar;

  Force()
  : Base() {}

  Force(const Force & other)
  : Base(other.toImplementation()) {}

  Force(const Base & other)
  : Base(other) {}

  Force(const Implementation & other)
  : Base(other) {}

  virtual ~Force() {}
};




template<typename PrimType_>
class Force3 : public vector::eigen_impl::Vector3<PrimType_>
{
 private:
  typedef vector::eigen_impl::Vector3<PrimType_> Base;

 public:
  typedef typename Base::Implementation Implementation;
  typedef PrimType_ Scalar;

  Force3()
  : Base() {}

  Force3(const Force3 & other)
  : Base(other.toImplementation()) {}

  Force3(const Base & other)
  : Base(other) {}

  Force3(const Implementation & other)
  : Base(other) {}

  Force3(Scalar x, Scalar y, Scalar z)
  : Base(x, y, z) {}

  virtual ~Force3() {}
};


typedef Force3<double> Force3D;
typedef Force3<float>  Force3F;


} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr





#endif /* KINDR_PHYS_QUANT_FORCE_HPP_ */
