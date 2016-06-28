function visualizeRotationFromAngleAxis(A_k_AB, theta)
% VISUALIZEROTATIONFROMANGLEAXIS visualizes the effect of rotating a
% coordinate A to frame B induced by a rotation represented by the
% angle-axis parametrization.
%
% Author(s): Dario Bellicoso

% Build the rotation vector from the angle-axis representation
A_phi_AB = A_k_AB*theta;

% Visualize the orientation
visualizeRotationFromRotationVector(A_phi_AB);

end