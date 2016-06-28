% LINEARIZATIONEXAMPLE demonstrates the use of the numerical_jacobian and
% the visualizeLinearization scripts.
%
% Author(s): Dario Bellicoso


% Define a nonlinear (possibly time variant) function handle for the 1D and
% the 2D case
fcn_1d = @(t,x)(x.^(2));
fcn_2d = @(t,x)(x(1).^2 + x(2).^2);

% Define the point around which to linearize
t0 = 0;
x0_1d = 2*rand - 1;
x0_2d = [2*rand - 1; 2*rand - 1];


% Compute the numerical jacobian using the built in method
thr_1d = 1e-6*ones(length(x0_1d),1);
thr_2d = 1e-6*ones(length(x0_2d),1);
dfdx_1d_matlab = numjac(fcn_1d, t0, x0_1d, fcn_1d(t0, x0_1d), thr_1d, [], 0);
dfdx_2d_matlab = numjac(fcn_2d, t0, x0_2d, fcn_2d(t0, x0_2d), thr_2d, [], 0);

% Compute the numerical jacobian using a custom method
dfdx_1d_custom = numerical_jacobian(fcn_1d, t0, x0_1d);
dfdx_2d_custom = numerical_jacobian(fcn_2d, t0, x0_2d);

% Check validity of custom jacobian computation
disp('Difference between 1d jacobians: ');
disp(norm(dfdx_1d_matlab-dfdx_1d_custom));

disp('Difference between 2d jacobians: ');
disp(norm(dfdx_2d_matlab-dfdx_2d_custom));

% Plot the nonlinear function and its linearization
visualizeLinearization(fcn_1d, t0, x0_1d);
visualizeLinearization(fcn_2d, t0, x0_2d);
