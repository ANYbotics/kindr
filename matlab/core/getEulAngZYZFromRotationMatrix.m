function ph = getEulAngZYZFromRotationMatrix(C)
% GETEULANGZYXFROMROTATIONMATRIX(C) extracts ZYZ Euler angles from a
% rotation matrix.
%
% Author(s): Dario Bellicoso

z1 = atan2(C(2,3),C(1,3));
y = atan2(sqrt(C(1,3)^2+C(2,3)^2), C(3,3));
z2 = atan2(C(3,2),-C(3,1));

ph = [z1 y z2]';
