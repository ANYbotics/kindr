function Cd = boxPlus(C, delta)
% BOXPLUS(C, delta) performs a global perturbation of a rotation matrix C
% with a rotational vector delta.
%
% Author(s): Dario Bellicoso

Cd = expm(skew(delta))*C;

end