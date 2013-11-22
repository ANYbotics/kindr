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
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
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
