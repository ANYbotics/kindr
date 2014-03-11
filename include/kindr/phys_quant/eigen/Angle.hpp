/*
 * Angle.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_ANGLE_HPP_
#define KINDR_PHYS_QUANT_EIGEN_ANGLE_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Angle-Vector
template <typename PrimType_, int Dimension_>
using Angle = vector::eigen_impl::Vector<phys_quant::PhysicalType::Angle, PrimType_, Dimension_>;
//! \brief 3D-Angle-Vector with primitive type double
typedef Angle<double, 3> Angle3D;
//! \brief 3D-Angle-Vector with primitive type float
typedef Angle<float,  3> Angle3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_ANGLE_HPP_ */
