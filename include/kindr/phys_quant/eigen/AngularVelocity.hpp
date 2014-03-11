/*
 * AngularVelocity.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_ANGULARVELOCITY_HPP_
#define KINDR_PHYS_QUANT_EIGEN_ANGULARVELOCITY_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief AngularVelocity-Vector
template <typename PrimType_, int Dimension_>
using AngularVelocity = vector::eigen_impl::Vector<phys_quant::PhysicalType::AngularVelocity, PrimType_, Dimension_>;
//! \brief 3D-AngularVelocity-Vector with primitive type double
typedef AngularVelocity<double, 3> AngularVelocity3D;
//! \brief 3D-AngularVelocity-Vector with primitive type float
typedef AngularVelocity<float,  3> AngularVelocity3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_ANGULARVELOCITY_HPP_ */
