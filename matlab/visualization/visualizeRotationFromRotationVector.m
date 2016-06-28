function visualizeRotationFromRotationVector(A_phi_AB)
% VISUALIZEROTATIONFROMROTATIONVECTOR visualizes the effect of rotating a
% coordinate A to frame B induced by the rotation vector A_phi_AB.
%
% Author(s): Dario Bellicoso

% Setup figure
h = figure();
view(3);
grid on;
axis equal;

% Map the rotation vector to the rotation matrix in SO(3)
C_AB = mapRotationVectorToRotationMatrix(A_phi_AB);

% Visualize the fixed and rotated frames
visualizeCoordinateSystem(eye(3), h, 'A');
visualizeCoordinateSystem(C_AB, h, 'B');

% Visualize the rotation vector
quiver3(0, 0, 0, A_phi_AB(1),A_phi_AB(2),A_phi_AB(3),'k');

% Label the rotation vector
addLabelToVector(h, A_phi_AB, '{}_A \varphi_{AB}');

end