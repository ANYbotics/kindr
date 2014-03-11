/*
 * Jerk.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_JERK_HPP_
#define KINDR_PHYS_QUANT_EIGEN_JERK_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vectors/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief Jerk-Vector
template <typename PrimType_, int Dimension_>
using Jerk = vectors::eigen_impl::Vector<phys_quant::PhysicalType::Jerk, PrimType_, Dimension_>;
//! \brief 3D-Velocity-Vector with primitive type double
typedef Jerk<double, 3> Jerk3D;
//! \brief 3D-Velocity-Vector with primitive type float
typedef Jerk<float,  3> Jerk3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_JERK_HPP_ */
