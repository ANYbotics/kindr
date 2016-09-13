function R = mapEulerAnglesZXZToRotationMatrix(angles)
% MAPEULERANGLESZXZTOROTATIONMATRIX(angles) maps a set of Euler angles to a
% rotation matrix in SO(3). The Euler angles represent a set successive
% rotations around Z-X'-Z''.
%
% Author(s): Dario Bellicoso

z1 = angles(1);
x  = angles(2);
z2 = angles(3);

R = getRotationMatrixZ(z1)*getRotationMatrixX(x)*getRotationMatrixZ(z2);

if isa(angles, 'sym')
    R = simplify(R);
end

end