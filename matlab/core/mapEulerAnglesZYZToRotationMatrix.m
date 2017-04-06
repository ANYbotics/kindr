function R = mapEulerAnglesZYZToRotationMatrix(angles)
% MAPEULERANGLESZYZTOROTATIONMATRIX(angles) maps a set of Euler angles to a
% rotation matrix in SO(3). The Euler angles represent a set successive
% rotations around Z-Y'-Z''.
%
% Author(s): Dario Bellicoso

z1 = angles(1);
y  = angles(2);
z2 = angles(3);

R = getRotationMatrixZ(z1)*getRotationMatrixY(y)*getRotationMatrixZ(z2);

if isa(angles, 'sym')
    R = simplify(R);
end

end