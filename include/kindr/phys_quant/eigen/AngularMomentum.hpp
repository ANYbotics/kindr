/*
 * AngularMomentum.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_ANGULARMOMENTUM_HPP_
#define KINDR_PHYS_QUANT_EIGEN_ANGULARMOMENTUM_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief AngularMomentum-Vector
template <typename PrimType_, int Dimension_>
using AngularMomentum = vector::eigen_impl::Vector<phys_quant::PhysicalType::AngularMomentum, PrimType_, Dimension_>;
//! \brief 3D-AngularMomentum-Vector with primitive type double
typedef AngularMomentum<double, 3> AngularMomentum3D;
//! \brief 3D-AngularMomentum-Vector with primitive type float
typedef AngularMomentum<float,  3> AngularMomentum3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_ANGULARMOMENTUM_HPP_ */
