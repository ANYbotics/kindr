function [th, n] = getAngleAxisFromRotationMatrix(C)
% GETANGLEAXISFROMROTATIONMATRIX(C) extracts an angle and an axis from a
% rotation matrix C.
%
% Author(s): Dario Bellicoso

th = acos(0.5*(C(1,1)+C(2,2)+C(3,3)-1));
n = 1/(2*sin(th))*[C(3,2)-C(2,3); 
                   C(1,3)-C(3,1);
                   C(2,1)-C(1,2)];

end