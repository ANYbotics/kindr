/*
 * Copyright (c) 2013, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/
#pragma once

#include "kindr/common/common.hpp"
#include <kindr/phys_quant/PhysicalType.hpp>
#include <kindr/vectors/Vector.hpp>

namespace kindr {

//! \brief Acceleration-Vector
template <typename PrimType_, int Dimension_>
using Acceleration = Vector<PhysicalType::Acceleration, PrimType_, Dimension_>;
//! \brief 3D-Acceleration-Vector with primitive type double
typedef Acceleration<double, 3> Acceleration3D;
//! \brief 3D-Acceleration-Vector with primitive type float
typedef Acceleration<float,  3> Acceleration3F;

//! \brief AngularAcceleration-Vector
template <typename PrimType_, int Dimension_>
using AngularAcceleration = Vector<PhysicalType::AngularAcceleration, PrimType_, Dimension_>;
//! \brief 3D-AngularAcceleration-Vector with primitive type double
typedef AngularAcceleration<double, 3> AngularAcceleration3D;
//! \brief 3D-AngularAcceleration-Vector with primitive type float
typedef AngularAcceleration<float,  3> AngularAcceleration3F;

//! \brief AngularJerk-Vector
template <typename PrimType_, int Dimension_>
using AngularJerk = Vector<PhysicalType::AngularJerk, PrimType_, Dimension_>;
//! \brief 3D-Angle-Vector with primitive type double
typedef AngularJerk<double, 3> AngularJerk3D;
//! \brief 3D-Angle-Vector with primitive type float
typedef AngularJerk<float,  3> AngularJerk3F;

//! \brief AngularVelocity-Vector
template <typename PrimType_, int Dimension_>
using AngularVelocity = Vector<PhysicalType::AngularVelocity, PrimType_, Dimension_>;
//! \brief 3D-Angle-Vector with primitive type double
typedef AngularVelocity<double, 3> AngularVelocity3D;
//! \brief 3D-Angle-Vector with primitive type float
typedef AngularVelocity<float,  3> AngularVelocity3F;

//! \brief AngularMomentum-Vector
template <typename PrimType_, int Dimension_>
using AngularMomentum = Vector<PhysicalType::AngularMomentum, PrimType_, Dimension_>;
//! \brief 3D-AngularMomentum-Vector with primitive type double
typedef AngularMomentum<double, 3> AngularMomentum3D;
//! \brief 3D-AngularMomentum-Vector with primitive type float
typedef AngularMomentum<float,  3> AngularMomentum3F;

//! \brief Force-Vector
template <typename PrimType_, int Dimension_>
using Force = Vector<PhysicalType::Force, PrimType_, Dimension_>;
//! \brief 3D-Force-Vector with primitive type double
typedef Force<double, 3> Force3D;
//! \brief 3D-Force-Vector with primitive type float
typedef Force<float,  3> Force3F;

//! \brief Jerk-Vector
template <typename PrimType_, int Dimension_>
using Jerk = Vector<PhysicalType::Jerk, PrimType_, Dimension_>;
//! \brief 3D-Velocity-Vector with primitive type double
typedef Jerk<double, 3> Jerk3D;
//! \brief 3D-Velocity-Vector with primitive type float
typedef Jerk<float,  3> Jerk3F;

//! \brief Momentum-Vector
template <typename PrimType_, int Dimension_>
using Momentum = Vector<PhysicalType::Momentum, PrimType_, Dimension_>;
//! \brief 3D-Momentum-Vector with primitive type double
typedef Momentum<double, 3> Momentum3D;
//! \brief 3D-Momentum-Vector with primitive type float
typedef Momentum<float,  3> Momentum3F;

//! \brief Position-Vector
template <typename PrimType_, int Dimension_>
using Position = Vector<PhysicalType::Position, PrimType_, Dimension_>;
//! \brief 3D-Position-Vector with primitive type double
typedef Position<double, 3> Position3D;
//! \brief 3D-Position-Vector with primitive type float
typedef Position<float,  3> Position3F;

//! \brief Torque-Vector
template <typename PrimType_, int Dimension_>
using Torque = Vector<PhysicalType::Torque, PrimType_, Dimension_>;
//! \brief 3D-Torque-Vector with primitive type double
typedef Torque<double, 3> Torque3D;
//! \brief 3D-Torque-Vector with primitive type float
typedef Torque<float,  3> Torque3F;

//! \brief Vector without type (e.g. normal vector)
template <typename PrimType_, int Dimension_>
using VectorTypeless = Vector<PhysicalType::Typeless, PrimType_, Dimension_>;
//! \brief 3D-Unitless-Vector with primitive type double
typedef VectorTypeless<double, 3> VectorTypeless3D;
//! \brief 3D-Unitless-Vector with primitive type float
typedef VectorTypeless<float,  3> VectorTypeless3F;

//! \brief Vector without type (e.g. normal vector)
template <typename PrimType_, int Dimension_>
using Time = Vector<PhysicalType::Time, PrimType_, Dimension_>;
//! \brief 3D-Unitless-Vector with primitive type double
typedef Time<double, 3> Time3D;
//! \brief 3D-Unitless-Vector with primitive type float
typedef Time<float,  3> Time3F;

//! \brief Vector without type (e.g. normal vector)
template <typename PrimType_, int Dimension_>
using Velocity = Vector<PhysicalType::Velocity, PrimType_, Dimension_>;
//! \brief 3D-Unitless-Vector with primitive type double
typedef Velocity<double, 3> Velocity3D;
//! \brief 3D-Unitless-Vector with primitive type float
typedef Velocity<float,  3> Velocity3F;

} // namespace
