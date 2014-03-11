/*
 * AngularJerk.hpp
 *
 *  Created on: Mar 11, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_EIGEN_ANGULARJERK_HPP_
#define KINDR_PHYS_QUANT_EIGEN_ANGULARJERK_HPP_

#include "kindr/phys_quant/PhysicalType.hpp"
#include "kindr/vector/VectorEigen.hpp"


namespace kindr {
namespace phys_quant {
namespace eigen_impl {

//! \brief AngularJerk-Vector
template <typename PrimType_, int Dimension_>
using AngularJerk = vector::eigen_impl::Vector<phys_quant::PhysicalType::AngularJerk, PrimType_, Dimension_>;
//! \brief 3D-Angle-Vector with primitive type double
typedef AngularJerk<double, 3> AngularJerk3D;
//! \brief 3D-Angle-Vector with primitive type float
typedef AngularJerk<float,  3> AngularJerk3F;

} // namespace eigen_impl
} // namespace phys_quant
} // namespace kindr


#endif /* KINDR_PHYS_QUANT_EIGEN_ANGULARJERK_HPP_ */
