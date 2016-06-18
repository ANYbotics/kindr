% function A=skew(a)
% -> generates a skew-symmetric matrix from vector a.
% 
% INPUT:
%   a         3x1- or 1x3-vector
%
% OUTPUT:
%   A         3x3 skew-symmetric matrix
%
% proNEu: tool for symbolic EoM derivation
% Copyright (C) 2011  Marco Hutter, Christian Gehring
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
function A=skew(a)
A = [0      -a(3)  a(2);
     a(3)     0   -a(1);
     -a(2)  a(1)     0];