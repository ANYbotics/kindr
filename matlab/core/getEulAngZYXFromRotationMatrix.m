function ph = getEulAngZYXFromRotationMatrix(C)
% GETEULANGZYXFROMROTATIONMATRIX(C) extracts ZYX Euler angles from a
% rotation matrix.
%
% Author(s): Dario Bellicoso

z = atan2(C(2,1),C(1,1));
y = atan2(-C(3,1), sqrt(C(3,2)^2+C(3,3)^2));
x = atan2(C(3,2),C(3,3));

ph = [z y x]';
