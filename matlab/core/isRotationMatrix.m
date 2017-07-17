function [res, str] = isRotationMatrix(C)
% ISROTATIONMATRIX(C) checks if the 3x3 matrix C is an element of SO(3)
%
% Author(s): Dario Bellicoso

% Initialize output
res = true;
str = ' ';

% Check size
[m,n] = size(C);
if (m~=3 || n~=3)
    res = false;
    str = [str 'Checking rotation matrix: input is not 3x3.'];
end

% Check determinant
detC = det(C);
if ( (detC-1) > 1e-10)
    res = false;
    str = [str ' Determinant of C was not 1. det(C) = ' num2str(detC) '.'];
end

% Check orthonormality
if (norm( (C*C')-eye(3)) > 1e-10)
    res = false;
    str = [str ' C is not orthonormal.'];
end

end