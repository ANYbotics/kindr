function q = mapAngleAxisToQuaternion(th, r)
% MAPANGLEAXISTOQUATERNION(th, r) maps an angle-axis parametrization to a
% quaternion one, represented as q = [qw qx qy qz]'
%
% Author(s): Dario Bellicoso

q0 = cos(th/2);
qv = sin(th/2)*r(:);

q = [q0; 
     qv];

end