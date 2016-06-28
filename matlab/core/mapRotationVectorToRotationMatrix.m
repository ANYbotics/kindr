function C = mapRotationVectorToRotationMatrix(phi)
% MAPROTATIONVECTORTOROTATIONMATRIX maps a rotation vector phi in so(3) to 
% a rotation matrix in SO(3)
%
%  syntax: C = mapRotationVectorToRotationMatrix(phi)
%
% Author(s): Dario Bellicoso


% phi is a rotation vector which describes the rotation of a frame w.r.t.
% another one. The rotation matrix in SO(3) can be computed by using the
% exponential map from so(3) to SO(3)
th = norm(phi);

if (abs(th) < eps)
    C = eye(3)   +   skew(phi);
else
    C = eye(3)   +   sin(th)/th*skew(phi)   +   (1-cos(th))/(th^2)*skew(phi)^2;
end


end