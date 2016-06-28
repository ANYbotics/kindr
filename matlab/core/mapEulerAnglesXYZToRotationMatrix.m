function R = mapEulerAnglesXYZToRotationMatrix(angles)
% MAPEULERANGLESXYZTOROTATIONMATRIX(angles) maps a set of Euler angles to a
% rotation matrix in SO(3). The Euler angles represent a set successive
% rotations around X-Y'-Z''. This is equivalent to rotating around the
% fixed axes in Z-Y-X order.
%
% Author(s): Dario Bellicoso

x = angles(1);
y = angles(2);
z = angles(3);

R = getRotationMatrixX(x)*getRotationMatrixY(y)*getRotationMatrixZ(z);

if isa(angles, 'sym')
    R = simplify(R);
end

end