/*
 * example.cpp
 *
 *  Created on: Nov 14, 2013
 *      Author: gech
 */

#include "rm/quaternions/QuaternionEigen.hpp"
#include "rm/rotations/RotationEigen.hpp"

int main(int argc, char **argv){
  using namespace rm::rotations::eigen_implementation;
  using namespace rm::quaternions::eigen_implementation;
  using namespace Eigen;
  Vector3d I_r_0F(1.0, 0.0, 0.0);

  RotationQuaternionPD p_B_A(0.707107, 0.0, 0.0, 0.707107);
  RotationQuaternionPD p_C_B(EulerAnglesXyzPD(0.0, M_PI/2.0, 0.0));
  RotationQuaternionPD p_D_C(AngleAxisPD(M_PI/2.0, Vector3d(1.0,0.0,0.0)));

  // Composition of rotations
  UnitQuaternionD p_D_B = p_D_C*p_C_B;
  RotationQuaternionPD p_D_A = p_D_C*p_C_B*p_B_A;
  EulerAnglesZyxPD rpy_D_A(M_PI/2.0, M_PI/2.0, M_PI/2.0);

  std::cout << "Quaternion: " << p_D_A << std::endl;
  std::cout << "Quaternion from rpy angles: " << RotationQuaternionPD(rpy_D_A) << std::endl;

  // Rotate vector
  std::cout << "Rotated vector: " << p_D_A.rotate(I_r_0F).transpose() << std::endl;
  std::cout << "Rotated vector: " << EulerAnglesXyzPD(M_PI/2.0, M_PI/2.0, M_PI/2.0).rotate(I_r_0F).transpose() << std::endl;


  if (p_D_A == RotationQuaternionPD(-p_D_A.w(), -p_D_A.x(), -p_D_A.y(), -p_D_A.z()) ){
    std::cout << "p_C_A and -p_C_A correspond to the same rotation." <<  std::endl;
  }
  return 0;
}
