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
C_IB_zyx = simplify(C_z*C_y*C_x)

% time derivative of rotation matrix
dC_IB_zyx = dMATdt( C_IB_zyx, zyx, dzyx )

% local angular velocity
B_w_IB = simplify(unskew(C_IB_zyx'*dC_IB_zyx))

% It must hold 
%  w = B_w_IB  or f := w - B_w_IB = 0 
f = w-B_w_IB;

% B_w_IB = E_zyx*dzyx;
res1 = solve(f, w1, w2, w3);
E_zyx = [jacobian(simplify(res1.w1), dzyx)
         jacobian(simplify(res1.w2), dzyx)
         jacobian(simplify(res1.w3), dzyx)]
ccode(E_zyx,'file', 'tmp/E_zyx_local.hpp')

% time derivative of E_zyx
dE_zyx = dMATdt(E_zyx, zyx, dzyx)
ccode(dE_zyx,'file', 'tmp/dE_zyx_local.hpp')

% dzyx = E_zyx_inv*B_w_IB;
res2 = solve(f, dx, dy, dz);
E_zyx_inv = [jacobian(simplify(res2.dz), w)
            jacobian(simplify(res2.dy), w)
            jacobian(simplify(res2.dx), w)]
ccode(E_zyx_inv,'file', 'tmp/E_zyx_local_inv.hpp')

% time derivative of E_zyx_inv  
dE_zyx_inv = dMATdt(E_zyx_inv, zyx, dzyx)
ccode(dE_zyx_inv,'file', 'tmp/dE_zyx_local_inv.hpp')

% global angular velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I_w_IB = simplify(unskew(dC_IB_zyx*C_IB_zyx'))
f_global = w-I_w_IB;

% B_w_IB = E_zyx*dzyx;
res1_global = solve(f_global, w1, w2, w3);
E_zyx_global = [jacobian(simplify(res1_global.w1), dzyx)
              jacobian(simplify(res1_global.w2), dzyx)
              jacobian(simplify(res1_global.w3), dzyx)]
ccode(E_zyx_global,'file', 'tmp/E_zyx_global.hpp')

% time derivative of E_zyx
dE_zyx_global = dMATdt(E_zyx_global, zyx, dzyx)
ccode(dE_zyx_global,'file', 'tmp/dE_zyx_global.hpp')

% dzyx = E_zyx_inv*B_w_IB;
res2_global = solve(f_global, dz, dy, dx);
E_zyx_global_inv = [jacobian(simplify(res2_global.dz), w)
            jacobian(simplify(res2_global.dy), w)
            jacobian(simplify(res2_global.dx), w)]
ccode(E_zyx_global_inv,'file', 'tmp/E_zyx_global_inv.hpp')

% time derivative of E_zyx_inv  
dE_zyx_global_inv = dMATdt(E_zyx_global_inv, zyx, dzyx)
ccode(dE_zyx_global_inv,'file', 'tmp/dE_zyx_global_inv.hpp')


%% Euler Angles XYZ

xyz = [x, y, z]';
dxyz = [dx, dy, dz]';

% rotation matrix
C_IB_xyz = simplify(C_x*C_y*C_z)

% time derivative of rotation matrix
dC_IB_xyz = dMATdt( C_IB_xyz, xyz, dxyz )

% local angular velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
B_w_IB = simplify(unskew(C_IB_xyz'*dC_IB_xyz))

% It must hold 
%  w = B_w_IB  or f := w - B_w_IB = 0 
f_local = w-B_w_IB;

% B_w_IB = E_zyx*dzyx;
res1_local = solve(f_local, w1, w2, w3);
E_xyz_local = [jacobian(simplify(res1_local.w1), dxyz)
              jacobian(simplify(res1_local.w2), dxyz)
              jacobian(simplify(res1_local.w3), dxyz)]
ccode(E_xyz_local,'file', 'tmp/E_xyz_local.hpp')

% time derivative of E_zyx
dE_xyz_local = dMATdt(E_xyz_local, xyz, dxyz)
ccode(dE_xyz_local,'file', 'tmp/dE_xyz_local.hpp')

% dzyx = E_zyx_inv*B_w_IB;
res2_local = solve(f_local, dx, dy, dz);
E_xyz_local_inv = [jacobian(simplify(res2_local.dx), w)
            jacobian(simplify(res2_local.dy), w)
            jacobian(simplify(res2_local.dz), w)]
ccode(E_xyz_local_inv,'file', 'tmp/E_xyz_local_inv.hpp')

% time derivative of E_zyx_inv  
dE_xyz_local_inv = dMATdt(E_xyz_local_inv, xyz, dxyz)
ccode(dE_xyz_local_inv,'file', 'tmp/dE_xyz_local_inv.hpp')

% global angular velocity
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

I_w_IB = simplify(unskew(dC_IB_xyz*C_IB_xyz'))
f_global = w-I_w_IB;

% B_w_IB = E_zyx*dzyx;
res1_global = solve(f_global, w1, w2, w3);
E_xyz_global = [jacobian(simplify(res1_global.w1), dxyz)
              jacobian(simplify(res1_global.w2), dxyz)
              jacobian(simplify(res1_global.w3), dxyz)]
ccode(E_xyz_global,'file', 'tmp/E_xyz_global.hpp')

% time derivative of E_zyx
dE_xyz_global = dMATdt(E_xyz_global, xyz, dxyz)
ccode(dE_xyz_global,'file', 'tmp/dE_xyz_global.hpp')

% dzyx = E_zyx_inv*B_w_IB;
res2_global = solve(f_global, dx, dy, dz);
E_xyz_global_inv = [jacobian(simplify(res2_global.dx), w)
            jacobian(simplify(res2_global.dy), w)
            jacobian(simplify(res2_global.dz), w)]
ccode(E_xyz_global_inv,'file', 'tmp/E_xyz_global_inv.hpp')

% time derivative of E_zyx_inv  
dE_xyz_global_inv = dMATdt(E_xyz_global_inv, xyz, dxyz)
ccode(dE_xyz_global_inv,'file', 'tmp/dE_xyz_global_inv.hpp')