/*
 * Acceleration.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_ACCELERATION_HPP_
#define KINDR_PHYS_QUANT_EIGEN_ACCELERATION_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vectors/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Acceleration-Vector
template <typename PrimType_, int Dimension_>
using Acceleration = vectors::eigen_impl::Vector<phys_quant::PhysicalType::Acceleration, PrimType_, Dimension_>;
//! \brief 3D-Acceleration-Vector with primitive type double
typedef Acceleration<double, 3> Acceleration3D;
//! \brief 3D-Acceleration-Vector with primitive type float
typedef Acceleration<float,  3> Acceleration3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_ACCELERATION_HPP_ */
