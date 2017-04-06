function ph = getEulAngZXZFromRotationMatrix(C)
% GETEULANGZYXFROMROTATIONMATRIX(C) extracts ZXZ Euler angles from a
% rotation matrix.
%
% Author(s): Dario Bellicoso

z1 = atan2(C(1,3),-C(2,3));
x = atan2(sqrt(C(1,3)^2+C(2,3)^2), C(3,3));
z2 = atan2(C(3,1),C(3,2));

ph = [z1 x z2]';
