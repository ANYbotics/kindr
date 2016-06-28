function H = getMapEulAngZYXDiffToAngVelInBaseFrame(angles)
% GETMAPEULANGZYXDIFFTOANGVELINBASEFRAME(angles) returns the 3x3 matrix 
% that maps the time derivative of ZYX Euler angles to angular velocity in
% base frame.
%
% Author(s): Dario Bellicoso

z = angles(1);
y = angles(2);

% The mapping is computed as:
% [B_w_IBx]                        [dz]
% [B_w_IBy] = R_BI * E([z,y,x]') * [dy]
% [B_w_IBz]                        [dx]
%
% H = R_BI * E([z,y,x]')
%
R_IB = mapEulerAnglesZYXToRotationMatrix(angles);
H = R_IB' * [0   -sin(z)    cos(y)*cos(z);
             0    cos(z)    cos(y)*sin(z)
             1    0        -sin(y)];

if (isa(angles, 'sym'))
    H = simplify(H);
end

end