/*
 * AngularAcceleration.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_ANGULARACCELERATION_HPP_
#define KINDR_PHYS_QUANT_EIGEN_ANGULARACCELERATION_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief AngularAcceleration-Vector
template <typename PrimType_, int Dimension_>
using AngularAcceleration = vector::eigen_impl::Vector<phys_quant::PhysicalType::AngularAcceleration, PrimType_, Dimension_>;
//! \brief 3D-AngularAcceleration-Vector with primitive type double
typedef AngularAcceleration<double, 3> AngularAcceleration3D;
//! \brief 3D-AngularAcceleration-Vector with primitive type float
typedef AngularAcceleration<float,  3> AngularAcceleration3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_ANGULARACCELERATION_HPP_ */
