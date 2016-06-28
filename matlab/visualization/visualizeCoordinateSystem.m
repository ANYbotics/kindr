function [h, frameHandle] = visualizeCoordinateSystem(C_AB, h, name)
% VISUALIZECOORDINATESYSTEM visualizes a Cartesian coordinate system which
% is rotated according to the rotation matrix C_AB w.r.t. a fixed frame.
% The visualization will update an existing figure with handle h.
%
% Author(s): Dario Bellicoso

% Check inputs
[res, str] = isRotationMatrix(C_AB);
if (res == 0)
    msg = ['C_AB is not a valid rotation matrix! Cause: ', str];
    error(msg);
end

if nargin == 1
    h = axes();
    useName = 0;
elseif nargin == 2
    axes(h);
    useName = 0;
else
    useName = 1;
    name = ['_' name];
end
hold on;

B_u_BX = [1 0 0]';
B_u_BY = [0 1 0]';
B_u_BZ = [0 0 1]';

A_u_BX = C_AB*B_u_BX;
A_u_BY = C_AB*B_u_BY;
A_u_BZ = C_AB*B_u_BZ;

frameHandle(1) = quiver3(0, 0, 0, A_u_BX(1), A_u_BX(2), A_u_BX(3), 'r', 'LineWidth', 1);
frameHandle(2) = quiver3(0, 0, 0, A_u_BY(1), A_u_BY(2), A_u_BY(3), 'g', 'LineWidth', 1);
frameHandle(3) = quiver3(0, 0, 0, A_u_BZ(1), A_u_BZ(2), A_u_BZ(3), 'b', 'LineWidth', 1);

if useName
    [h, textHandle] = addLabelToVector(h, A_u_BX, ['x' name]);
    frameHandle = [frameHandle textHandle];
    
    [h, textHandle] = addLabelToVector(h, A_u_BY, ['y' name]);
    frameHandle = [frameHandle textHandle];
    
    [h, textHandle] = addLabelToVector(h, A_u_BZ, ['z' name]);
    frameHandle = [frameHandle textHandle];
end

end