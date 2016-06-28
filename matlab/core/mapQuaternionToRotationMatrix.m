function C = mapQuaternionToRotationMatrix(q)
% MAPQUATERNIONTOROTATIONMATRIX maps a quaternion q to SO(3)
% The function MAPQUATERNIONTOROTATIONMATRIX(q) maps a quaternion 
% q = [qw qx qy qz]' to a rotation matrix C.
%
% Author(s): Dario Bellicoso

q0 = q(1);
qv = q(2:4);

sQv = skew(qv);

C = (2*q0^2-1)*eye(3) + 2*q0*sQv + 2*(qv*qv');

end