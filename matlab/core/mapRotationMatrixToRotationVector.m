function phi = mapRotationMatrixToRotationVector(C)
% MAPROTATIONMATRIXTOROTATIONVECTOR maps a rotation matrix in SO(3) to 
% a rotation vector phi in so(3).
% 
%
%  syntax: phi = mapRotationMatrixToRotationVector(C)
%
% Author(s): Dario Bellicoso

phi = unskew(logm(C));

end