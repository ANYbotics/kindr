/*
 * example.cpp
 *
 *  Created on: Nov 14, 2013
 *      Author: gech
 */

#include "rm/rotations/RotationEigen.hpp"

int main(int argc, char **argv){
  using namespace rm::rotations::eigen_implementation;
  using namespace Eigen;
  Vector3d I_r_0F(1.0, 0.0, 0.0);

  RotationQuaternionD p_B_A(0.707107, 0.0, 0.0, 0.707107);
  RotationQuaternionD p_C_B(EulerAnglesXYZD(0.0, M_PI/2.0, 0.0));
  RotationQuaternionD p_D_C(AngleAxisD(M_PI/2.0, Vector3d(1.0,0.0,0.0)));

  // Composition of rotations
  RotationQuaternionD p_D_A = p_D_C*p_C_B*p_B_A;
  EulerAnglesZYXD rpy_D_A(M_PI/2.0, M_PI/2.0, M_PI/2.0);

  std::cout << "Quaternion: " << p_D_A << std::endl;
  std::cout << "Quaternion from rpy angles: " << RotationQuaternionD(rpy_D_A) << std::endl;

  // Rotate vector
  std::cout << "Rotated vector: " << p_D_A.rotate(I_r_0F).transpose() << std::endl;
  std::cout << "Rotated vector: " << EulerAnglesXYZD(M_PI/2.0, M_PI/2.0, M_PI/2.0).rotate(I_r_0F).transpose() << std::endl;


  if (p_D_A == RotationQuaternionD(-p_D_A.w(), -p_D_A.x(), -p_D_A.y(), -p_D_A.z()) ){
    std::cout << "p_C_A and -p_C_A correspond to the same rotation." <<  std::endl;
  }
  return 0;
}
