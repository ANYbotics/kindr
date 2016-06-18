% function dA = dMATdt (A,q,dq)
%
% -> total time derivation of a matrix or a vector A(q).
% dA(q)/dt = dA/dq*dq, 
% where dq is vector q derived with respect to time t.
%
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
function [ dA ] = dMATdt( A, q, dq )

dA = sym(zeros(size(A)));
for i=1:size(A,1)
    for j=1:size(A,2)
        dA(i,j) = jacobian(A(i,j),q)*dq;
    end
end
end

