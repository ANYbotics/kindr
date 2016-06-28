function [axisHandle, labelHandle] = addLabelToVector(axisHandle, vec, str)
% ADDLABELTOVECTOR(axisHandle, vec, str) adds a text label to a vector in a
% figure.
%
% Author(s): Dario Bellicoso

% Make h the current figure
axes(axisHandle);

% Generate a random displacement vector. This helps to avoid overlapping of
% labels
deltaDisplacement = 2*rand(3,1)-1;
delta = 0.1*deltaDisplacement/norm(deltaDisplacement);

% Add the text
labelHandle = text(vec(1)+delta(1), vec(2)+delta(2), vec(3)+delta(3), ['$$' str '$$'], 'Interpreter', 'Latex');

end