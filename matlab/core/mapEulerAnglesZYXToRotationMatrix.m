function R = mapEulerAnglesZYXToRotationMatrix(angles)
% MAPEULERANGLESZYXTOROTATIONMATRIX(angles) maps a set of Euler angles to a
% rotation matrix in SO(3). The Euler angles represent a set successive
% rotations around Z-Y'-X''. This is equivalent to rotating around the
% fixed axes in X-Y-Z order.
%
% Author(s): Dario Bellicoso

z = angles(1);
y = angles(2);
x = angles(3);

if isa(angles, 'sym')
    R = simplify(getRotationMatrixZ(z)*getRotationMatrixY(y)*getRotationMatrixX(x));
else
    R = getRotationMatrixZ(z)*getRotationMatrixY(y)*getRotationMatrixX(x);
end

end