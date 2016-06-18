%% Kindr 1.0.0
% Author(s): Christian Gehring

clear all, clc

%% Variables

syms x y z real;
syms dx dy dz real;
syms w1 w2 w3 real;

% angular velociy
w = [w1, w2, w3]';

%% Elementary Rotations

C_x = [1 0      0;
       0 cos(x) -sin(x);
       0 sin(x) cos(x)];

C_y = [cos(y)  0 sin(y);
         0     1   0;
       -sin(y) 0 cos(y)];
 
C_z = [cos(z)  -sin(z)  0;
       sin(z)   cos(z)  0;
         0        0     1];

%% Euler Angles ZYX

zyx = [z, y, x]';
dzyx = [dz, dy, dx]';


% rotation matrix
C_IB = simplify(C_x*C_y*C_z)

% time derivative of rotation matrix
dC_IB = dMATdt( C_IB, zyx, dzyx )

% local angular velocity
B_w_IB = simplify(unskew(C_IB'*dC_IB))

% It must hold 
%  w = B_w_IB  or f := w - B_w_IB = 0 
f = w-B_w_IB;

% B_w_IB = E_zyx*dzyx;
res1 = solve(f, w1, w2, w3);
E_zyx = [jacobian(simplify(res1.w1), dzyx)
         jacobian(simplify(res1.w2), dzyx)
         jacobian(simplify(res1.w3), dzyx)]
     
% time derivative of E_zyx
dE_zyx = dMATdt(E_zyx, zyx, dzyx)
ccode(dE_zyx,'file', 'tmp/dE_zyx.hpp')

% dzyx = E_zyx_inv*B_w_IB;
res2 = solve(f, dx, dy, dz);
E_zyx_inv = [jacobian(simplify(res2.dz), w)
            jacobian(simplify(res2.dy), w)
            jacobian(simplify(res2.dx), w)]
        
% time derivative of E_zyx_inv  
dE_zyx_inv = dMATdt(E_zyx_inv, zyx, dzyx)
ccode(dE_zyx_inv,'file', 'tmp/dE_zyx_inv.hpp')

%% Euler Angles XYZ

xyz = [x, y, z]';
dxyz = [dx, dy, dz]';

% rotation matrix
C_IB = simplify(C_z*C_y*C_x)

% time derivative of rotation matrix
dC_IB = dMATdt( C_IB, xyz, dxyz )

% local angular velocity
B_w_IB = simplify(unskew(C_IB'*dC_IB))

% It must hold 
%  w = B_w_IB  or f := w - B_w_IB = 0 
f = w-B_w_IB;

% B_w_IB = E_zyx*dzyx;
res1 = solve(f, w1, w2, w3);
E_xyz = [jacobian(simplify(res1.w1), dxyz)
         jacobian(simplify(res1.w2), dxyz)
         jacobian(simplify(res1.w3), dxyz)]
     
% time derivative of E_zyx
dE_xyz = dMATdt(E_xyz, xyz, dxyz)
ccode(dE_xyz,'file', 'tmp/dE_xyz.hpp')

% dzyx = E_zyx_inv*B_w_IB;
res2 = solve(f, dx, dy, dz);
E_xyz_inv = [jacobian(simplify(res2.dx), w)
            jacobian(simplify(res2.dy), w)
            jacobian(simplify(res2.dz), w)]
        
% time derivative of E_zyx_inv  
dE_xyz_inv = dMATdt(E_xyz_inv, xyz, dxyz)
ccode(dE_xyz_inv,'file', 'tmp/dE_xyz_inv.hpp')
