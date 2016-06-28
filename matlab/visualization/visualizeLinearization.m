function visualizeLinearization(fcn, t0, x0)
% VISUALIZELINEARIZATION(fcn, t0, x0) visualizes a function whos domain is
% scalar or bidimensional and its linearization around the tuple (t0, x0).
% The function is specified by a handle in the form f(t,x).
%
% Author(s): Dario Bellicoso

% Compute the numerical approximation of the function around (t0,x0)
dfdp = numerical_jacobian(fcn, t0, x0);

% Check the domain dimension
if (length(x0) == 1)
    % Scalar domain
    
    % Setup
    n = 1000;
    rangeLin = 0.5;
    range = 1;
    
    % Prepare a figure
    figure('name', 'Linearization of function with scalar domain');
    hold on; grid on;
    axis equal;
    
    % Domain for the input function
    x = linspace(x0-range, x0+range, n);
    
    % Domain for linearized function
    xlin = linspace(x0-rangeLin, x0+rangeLin, n);
    
    y = fcn(t0, x);
    ylin = fcn(t0, x0) + dfdp*(xlin-x0);

    plot(x, y, 'b');
    plot(xlin, ylin, 'r');

    scale = -0.5;
    quiver(x0, fcn(t0,x0), scale*dfdp, -scale, 'LineWidth', 2);
    plot(x0, fcn(t0,x0), 'rx');
    
elseif (length(x0) == 2)
    % 2d domain
    
    % Setup
    n = 200;
    rangeLin = 0.5;
    range = 2;
    
    % Prepare a figure
    figure('name', 'Linearization of function with bidimensional domain');
    hold on; grid on;
    axis equal;

    % Domain for the input function
    x = linspace(x0(1)-range, x0(1)+range, n);
    y = linspace(x0(2)-range, x0(2)+range, n);
    
    % Domain for linearized function
    xlin = linspace(x0(1)-rangeLin, x0(1)+rangeLin, n);
    ylin = linspace(x0(2)-rangeLin, x0(2)+rangeLin, n);

    [Xlin,Ylin] = meshgrid(xlin,ylin);
    [X,Y] = meshgrid(x,y);

    Z = zeros(n,n);
    Zlin = zeros(n,n);

    % Compute f(t,x) and its linearization
    for k=1:n
        for h=1:n
            Z(k,h) = fcn(0, [x(h),y(k)]);
            Zlin(k,h) = fcn(t0, x0) + dfdp*([xlin(h);ylin(k)]-x0);
        end
    end
   
    s1 = surf(X,Y,Z);
    set(s1, 'edgecolor','none');

    s2 = surf(Xlin,Ylin,Zlin, 2*Zlin);
    set(s2, 'edgecolor','none');

    scale = -0.5;
    quiver3(x0(1),x0(2),fcn(t0,x0),scale*dfdp(1),scale*dfdp(2),-scale, 'LineWidth', 2);
    plot3(x0(1),x0(2),fcn(t0,x0),'rx');
    view(3);
    
else
    % Cannot visualize a function of more than two variables
    disp('Dimension of domain is > 2.');
end


end