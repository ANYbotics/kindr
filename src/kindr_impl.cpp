/*
 * kindr_impl.cpp
 *
 *  Created on: Jun 13, 2016
 *      Author: gech
 */


#include "kindr/rotations/Rotation.hpp"
#include "kindr/rotations/RotationDiff.hpp"

int main(int argc, char **argv) {
  kindr::RotationMatrixD rotMatrix;
  kindr::RotationVectorD rotVector;
  kindr::RotationQuaternionD rotQuaternion;
  kindr::AngleAxisD angleAxis;
  kindr::EulerAnglesZyxD eulerAnglesZyx;
  kindr::EulerAnglesXyzD eulerAnglesXyz;

  kindr::RotationMatrixDiffD rotMatrixDiff;
  kindr::RotationQuaternionDiffD rotQuaternionDiff;
  kindr::EulerAnglesZyxDiffD eulerAnglesZyxDiff;
  kindr::EulerAnglesXyzDiffD eulerAnglesXyzDiff;
}
