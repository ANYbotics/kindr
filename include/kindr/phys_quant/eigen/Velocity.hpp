/*
 * Velocity.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_VELOCITY_HPP_
#define KINDR_PHYS_QUANT_EIGEN_VELOCITY_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vectors/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Velocity-Vector
template <typename PrimType_, int Dimension_>
using Velocity = vectors::eigen_impl::Vector<phys_quant::PhysicalType::Velocity, PrimType_, Dimension_>;
//! \brief 3D-Velocity-Vector with primitive type double
typedef Velocity<double, 3> Velocity3D;
//! \brief 3D-Velocity-Vector with primitive type float
typedef Velocity<float,  3> Velocity3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_VELOCITY_HPP_ */
