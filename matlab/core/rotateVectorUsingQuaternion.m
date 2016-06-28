function A_r = rotateVectorUsingQuaternion(q_AB, B_r)
% ROTATEVECTORUSINGQUATERNION(q_AB, B_r) projects the components of B_r to
% those of A_r.
%
% Author(s): Dario Bellicoso

q0 = q_AB(1);
qv = q_AB(2:4);
qv = qv(:);
sQv = skew(qv);

B_r = B_r(:);

A_r = (2*q0^2-1)*B_r + 2*q0*sQv*B_r + 2*qv*(qv'*B_r);

end