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

/*! \class PhysicalType
 * \brief Physical type of a vector.
 *
 * This enum class contains all possible physical type a vector can have in kindr.
 * \ingroup phys_quant
 */
enum class PhysicalType {
  Undefined, // 0

  Time, // 1
  Mass, // 2
  Inertia, // 3
  Energy, // 4

  Acceleration, // 5
  Velocity, // 6
  Length, // 7
  Jerk, // 8
  Force, // 9
  Momentum, // 10

  AngularAcceleration, // 11
  AngularVelocity, // 12
  Angle, // 13
  AngularJerk, // 14
  Torque, // 15
  AngularMomentum // 16
};


} // namespace phys_quant
} // namespace kindr





#endif /* KINDR_PHYS_QUANT_PHYSICALTYPE_HPP_ */
