function phi = mapRotationQuaternionToRotationVector(q)
% MAPROTATIONQUATERNIONTOROTATIONVECTOR(q) extracts a rotation vector from
% a unit quaternion.
%
% Author(s): Dario Bellicoso

th = q(1);
n = q(2:4);

if (norm(n) < eps)
    if (th>0)
        phi = 2*n;
    else
        phi = -2*n;
    end
else
    phi = 2*atan2(norm(n), th)*n/norm(n);
end

end