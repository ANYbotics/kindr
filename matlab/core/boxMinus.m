function delta = boxMinus(C1, C2)
% BOXMINUS(C1, C2) extracts the perturbation delta from two rotation
% matrices C1 and C2.
%
% Author(s): Dario Bellicoso

delta = unskew(logm(C1*C2'));

end