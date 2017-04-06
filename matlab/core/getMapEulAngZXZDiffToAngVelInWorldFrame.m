function E = getMapEulAngZXZDiffToAngVelInWorldFrame(angles)
% GETMAPEULANGZXZDIFFTOANGVELINWORLDFRAME(angles) returns the 3x3 matrix 
% that maps the time derivative of ZXZ Euler angles to angular velocity in
% world frame.
%
% Author(s): Dario Bellicoso

z1 = angles(1);
x  = angles(2);

% The mapping is computed as:
% [I_w_IBx]                   [dz1]
% [I_w_IBy] = E([z1,x,z2]') * [dx]
% [I_w_IBz]                   [dz2]
E = [0   cos(z1)   sin(x)*sin(z1);
     0   sin(z1)  -cos(z1)*sin(x)
     1   0         cos(x)];

if (isa(angles, 'sym'))
    E = simplify(E);
end

end