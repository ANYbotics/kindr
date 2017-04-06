function ph = getEulAngXYZFromRotationMatrix(C)
% GETEULANGXYZFROMROTATIONMATRIX(C) extracts XYZ Euler angles from a
% rotation matrix.
%
% Author(s): Dario Bellicoso

x = atan2(-C(2,3),C(3,3));
y = atan2(C(1,3), sqrt(C(1,1)^2+C(1,2)^2));
z = atan2(-C(1,2),C(1,1));

ph = [x y z]';
