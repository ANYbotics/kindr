function q = mapRotationMatrixToQuaternion(C)
% MAPROTATIONMATRIXTOQUATERNION(C) extracts a quaternion from a rotation
% matrix C.
%
% Author(s): Dario Bellicoso

[th, n] = getAngleAxisFromRotationMatrix(C);
q = mapAngleAxisToQuaternion(th, n);

end