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
  None,

  Time,
  Mass,
  Inertia,
  Energy,

  Length,
  Velocity,
  Acceleration,
  Force,
  Momentum,

  Angle,
  AngularVelocity,
  AngularAcceleration,
  Torque,
  AngularMomentum
};


} // namespace phys_quant
} // namespace kindr





#endif /* KINDR_PHYS_QUANT_PHYSICALTYPE_HPP_ */
