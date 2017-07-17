function [h, frameHandle] = visualizeHomogeneousTransform(T_AB, h, name, lineWidth)
% VISUALIZEHOMOGENEOUSTRANSFORM visualizes a Cartesian coordinate system
% which is rotated according to the rotation matrix C_AB w.r.t. a fixed
% frame. The visualization will update an existing axes with handle h or 
% create a new one.
%
% Author(s): Dario Bellicoso

% Check inputs
C_AB = T_AB(1:3, 1:3);
A_r_AB = T_AB(1:3,4);

[res, str] = isRotationMatrix(C_AB);
if (res == 0)
    msg = ['C_AB is not a valid rotation matrix! Cause: ', str];
    error(msg);
end

if nargin == 1
    h = gca();
    useName = 0;
    lineWidth = 1;
elseif nargin == 2
    axes(h);
    useName = 0;
    lineWidth = 1;
elseif nargin == 3
    axes(h);
    useName = 1;
    name = ['_' name];
    lineWidth = 1;
elseif nargin == 4
    axes(h);
    useName = 1;
    name = ['_' name];
else
    h = gca();
    C_AB = eye(3);
    name = '_I';
    useName = 1;
    lineWidth = 1;
end
hold on; grid on;

line_length = 0.05;
B_u_BX = line_length*[1 0 0]';
B_u_BY = line_length*[0 1 0]';
B_u_BZ = line_length*[0 0 1]';

A_u_BX = C_AB*B_u_BX;
A_u_BY = C_AB*B_u_BY;
A_u_BZ = C_AB*B_u_BZ;

frameHandle(1) = quiver3(A_r_AB(1), A_r_AB(2), A_r_AB(3), ...
                         A_u_BX(1), A_u_BX(2), A_u_BX(3), 0, ...
                         'r', 'LineWidth', lineWidth);
frameHandle(2) = quiver3(A_r_AB(1), A_r_AB(2), A_r_AB(3), ...
                         A_u_BY(1), A_u_BY(2), A_u_BY(3), 0, ...
                         'g', 'LineWidth', lineWidth);
frameHandle(3) = quiver3(A_r_AB(1), A_r_AB(2), A_r_AB(3), ...
                         A_u_BZ(1), A_u_BZ(2), A_u_BZ(3), ...
                         0, 'b', 'LineWidth', lineWidth);

if useName
    [h, textHandle] = addLabelToVector(h, A_u_BX, ['x' name]);
    frameHandle = [frameHandle textHandle];
    
    [h, textHandle] = addLabelToVector(h, A_u_BY, ['y' name]);
    frameHandle = [frameHandle textHandle];
    
    [h, textHandle] = addLabelToVector(h, A_u_BZ, ['z' name]);
    frameHandle = [frameHandle textHandle];
end

end