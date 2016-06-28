function dfdx = numerical_jacobian(fcn, t0, x0, delta)
%NUMERICAL_JACOBIAN(fcn, t0, x0, delta) Jacobian of a nonlinear function
%   dfdx = NUMERICAL_JACOBIAN(fcn, t0, x0, delta) returns the numerical
%   approximation of the jacobian of a function fcn(t,x) around the tuple
%   (t0, x0). The optional parameter delta is the variation size.
%
% Author(s): Dario Bellicoso

%% Setup
n = length(x0);
m = length(fcn(t0,x0));
dfdx = zeros(m,n);

if nargin < 4
    delta = 1e-5;
end

%% Numerical approximation

% Evaluate the columns of the jacobian of fcn(t,x)
dx = zeros(n,1);
for k=1:n
    % Perturb x(k) by delta
    dx(k) = delta;

    % Evaluate the k-th column of the jacobian through finite differences
    dfdx(:,k) =  (fcn(t0, x0+dx) - fcn(t0, x0-dx))/(2*delta);
    
    % Reset perturbation
    dx(k) = 0;
end

end