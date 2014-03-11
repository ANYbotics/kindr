/*
 * Torque.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_TORQUE_HPP_
#define KINDR_PHYS_QUANT_EIGEN_TORQUE_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vectors/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Torque-Vector
template <typename PrimType_, int Dimension_>
using Torque = vectors::eigen_impl::Vector<phys_quant::PhysicalType::Torque, PrimType_, Dimension_>;
//! \brief 3D-Torque-Vector with primitive type double
typedef Torque<double, 3> Torque3D;
//! \brief 3D-Torque-Vector with primitive type float
typedef Torque<float,  3> Torque3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_TORQUE_HPP_ */
