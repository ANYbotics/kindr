function E = getMapEulAngZYZDiffToAngVelInWorldFrame(angles)
% GETMAPEULANGZYZDIFFTOANGVELINWORLDFRAME(angles) returns the 3x3 matrix 
% that maps the time derivative of ZYZ Euler angles to angular velocity in
% world frame.
%
% Author(s): Dario Bellicoso

z = angles(1);
y = angles(2);

% The mapping is computed as:
% [I_w_IBx]                   [dz]
% [I_w_IBy] = E([z1,y,z2]') * [dy]
% [I_w_IBz]                   [dx]
E = [0   -sin(z)    cos(y)*cos(z);
     0    cos(z)    cos(y)*sin(z)
     1    0        -sin(y)];

if (isa(angles, 'sym'))
    E = simplify(E);
end

end