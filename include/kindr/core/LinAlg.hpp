/*
 * LinAlg.hpp
 *
 *  Created on: Nov 17, 2014
 *      Author: hannes
 */

#ifndef INCLUDE_KINDR_CORE_LINALG_HPP_
#define INCLUDE_KINDR_CORE_LINALG_HPP_

#include <Eigen/Core>

namespace kindr {
namespace core {

template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
using Eigen::Vector3f;
using Eigen::Vector3d;

template <typename Scalar, int Columns>
using Matrix3N = Eigen::Matrix<Scalar, 3, Columns>;

template <typename Scalar>
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;
using Eigen::Vector4f;
using Eigen::Vector4d;

template <typename Scalar, int Columns>
using Matrix4N = Eigen::Matrix<Scalar, 4, Columns>;

template <typename Scalar>
using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
typedef Vector6<float> Vector6f;
typedef Vector6<double> Vector6d;


template <typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Eigen::Matrix3f;
using Eigen::Matrix3d;

template <typename Scalar>
using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
using Eigen::Matrix4f;
using Eigen::Matrix4d;


}
}

#endif /* INCLUDE_KINDR_CORE_LINALG_HPP_ */
