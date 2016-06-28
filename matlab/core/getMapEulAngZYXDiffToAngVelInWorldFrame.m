function E = getMapEulAngZYXDiffToAngVelInWorldFrame(angles)
% GETMAPEULANGZYXDIFFTOANGVELINWORLDFRAME(angles) returns the 3x3 matrix 
% that maps the time derivative of ZYX Euler angles to angular velocity in
% world frame.
%
% Author(s): Dario Bellicoso

z = angles(1);
y = angles(2);

% The mapping is computed as:
% [I_w_IBx]                 [dz]
% [I_w_IBy] = E([z,y,x]') * [dy]
% [I_w_IBz]                 [dx]
E = [0   -sin(z)    cos(y)*cos(z);
     0    cos(z)    cos(y)*sin(z)
     1    0        -sin(y)];

if (isa(angles, 'sym'))
    E = simplify(E);
end

end