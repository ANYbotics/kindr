/*
 * PhysicalType.hpp
 *
 *  Created on: Feb 25, 2014
 *      Author: remod
 */

#ifndef KINDR_PHYS_QUANT_PHYSICALTYPE_HPP_
#define KINDR_PHYS_QUANT_PHYSICALTYPE_HPP_



namespace kindr {
namespace phys_quant {


enum class PhysicalType {
  Undefined,

  Time,
  Mass,
  Inertia,
  Energy,

  Length,
  Velocity,
  Acceleration,
  Jerk,
  Force,
  Momentum,

  Angle,
  AngularVelocity,
  AngularAcceleration,
  AngularJerk,
  Torque,
  AngularMomentum
};


} // namespace phys_quant
} // namespace kindr





#endif /* KINDR_PHYS_QUANT_PHYSICALTYPE_HPP_ */
