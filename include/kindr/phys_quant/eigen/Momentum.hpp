/*
 * Momentum.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_MOMENTUM_HPP_
#define KINDR_PHYS_QUANT_EIGEN_MOMENTUM_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vectors/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Momentum-Vector
template <typename PrimType_, int Dimension_>
using Momentum = vectors::eigen_impl::Vector<phys_quant::PhysicalType::Momentum, PrimType_, Dimension_>;
//! \brief 3D-Momentum-Vector with primitive type double
typedef Momentum<double, 3> Momentum3D;
//! \brief 3D-Momentum-Vector with primitive type float
typedef Momentum<float,  3> Momentum3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_MOMENTUM_HPP_ */
