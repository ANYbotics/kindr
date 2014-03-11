/*
 * Force.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_FORCE_HPP_
#define KINDR_PHYS_QUANT_EIGEN_FORCE_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Force-Vector
template <typename PrimType_, int Dimension_>
using Force = vector::eigen_impl::Vector<phys_quant::PhysicalType::Force, PrimType_, Dimension_>;
//! \brief 3D-Force-Vector with primitive type double
typedef Force<double, 3> Force3D;
//! \brief 3D-Force-Vector with primitive type float
typedef Force<float,  3> Force3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_FORCE_HPP_ */
