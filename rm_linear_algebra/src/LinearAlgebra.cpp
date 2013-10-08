/*
 * LinearAlgebra.cpp
 *
 *  Created on: Aug 22, 2013
 *      Author: Péter Fankhauser, Christian Gehring
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <rm/linear_algebra/LinearAlgebra.hpp>

namespace rm {
namespace linear_algebra {


Eigen::Matrix3d getSkewMatrixFromVector(const Eigen::Vector3d& vec) {
  Eigen::Matrix3d mat;
  mat << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
  return mat;
}

} // end namespace linear_algebra
} // end namespace rm
