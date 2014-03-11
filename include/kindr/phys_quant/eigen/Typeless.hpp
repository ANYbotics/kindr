/*
 * Typeless.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_TYPELESS_HPP_
#define KINDR_PHYS_QUANT_EIGEN_TYPELESS_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Vector without type (e.g. normal vector)
template <typename PrimType_, int Dimension_>
using VectorTypeless = vector::eigen_impl::Vector<phys_quant::PhysicalType::Typeless, PrimType_, Dimension_>;
//! \brief 3D-Unitless-Vector with primitive type double
typedef VectorTypeless<double, 3> VectorTypeless3D;
//! \brief 3D-Unitless-Vector with primitive type float
typedef VectorTypeless<float,  3> VectorTypeless3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_TYPELESS_HPP_ */
