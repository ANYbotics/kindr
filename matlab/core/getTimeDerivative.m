function dAdt = getTimeDerivative(A, x, dx)
% GETTIMEDERIVATIVE(A, x, dx) computes the time derivative of A(x(t))
%
% Author(s): Dario Bellicoso


[m,n] = size(A);

if isa(A, 'sym')
    dAdt = sym(zeros(m,n));
else
    dAdt = zeros(m,n);
end

% Make dx a column vector
dx = dx(:);

for k=1:m
    for h=1:n
        dAdt(k,h) = jacobian(A(k,h), x)*dx;
    end
end


if isa(A, 'sym')
    dAdt = simplify(dAdt);
end

end