function v = unskew(S)
% UNSKEW(S) extracts a vector v in R^3 from a skew matrix S(v).
%
% Author(s): Dario Bellicoso

v = [S(3,2); S(1,3); S(2,1)];

end