/*
 * Position.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_POSITION_HPP_
#define KINDR_PHYS_QUANT_EIGEN_POSITION_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Position-Vector
template <typename PrimType_, int Dimension_>
using Position = vector::eigen_impl::Vector<phys_quant::PhysicalType::Position, PrimType_, Dimension_>;
//! \brief 3D-Position-Vector with primitive type double
typedef Position<double, 3> Position3D;
//! \brief 3D-Position-Vector with primitive type float
typedef Position<float,  3> Position3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_POSITION_HPP_ */
