function animateAngularVelocityUsingEulerZYX(I_w_IB, rate, updateFrequency)
% ANIMATEANGULARVELOCITY(I_w_IB, rate, updateFrequency) generates an 
% animation showing the rotation of a coordinate system w.r.t. a fixed one.
%
% Author(s): Dario Bellicoso

% Provide a default scaling for the angular velocity
if nargin < 2
    rate = 1;
end

% Provide a default redraw frequency
if nargin < 3
    updateFrequency = 25; % Hz
end

% Setup figure
limit = 1.5;
limitsX = [-limit limit];
limitsY = [-limit limit];
limitsZ = [-limit limit];
figure();
axHandle = axes();
view(3);
hold on; grid on; axis equal;

% Get scalar angular velocity and rotation direction
I_w_IB = I_w_IB*rate;
w = norm(I_w_IB);
w_vec = I_w_IB/w;

% Animate for t_f seconds
t_f = 30.0;

% Define how many 'old' coordinate frame to show. This emulates fading out
% of the coordinate frame animation. Use 1 to just show the latest update
traceMemory = 1;

% Discrete steps to jump when visualizing an older coordinate frame
skipSteps = 5;

% Visualize the coordinate frame unit vectors and get handles to them
[axHandle, inertialFrameHandle] = visualizeCoordinateSystem(eye(3), axHandle, 'I');

% Draw an ellipsoid as a rigid body representation
bodyScale = 0.5;
[x, y, z] = ellipsoid(0, 0, 0, bodyScale*1.0, bodyScale*0.75, bodyScale*0.5, 50);
bodyHandle = surf(x, y, z, 2*z);
set(bodyHandle, 'edgecolor','none');

% Initialize containers
transforms = cell(traceMemory,1);
handles = cell(traceMemory,1);
colorNorms = linspace(0, 1, traceMemory);
colorNorms = fliplr(colorNorms);

C_IB = cell(traceMemory,1);
for k=1:traceMemory
    C_IB{k} = eye(3);
end

for k=1:traceMemory
    % Use axis labels only for the first rotation set
    if k==1
        [axHandle, frameHandle] = visualizeCoordinateSystem(eye(3), axHandle, 'B');
    else
        [axHandle, frameHandle] = visualizeCoordinateSystem(eye(3), axHandle);
    end
    
    % Decrease the color norm for older rotation sets
    for kHandle=1:3
        handleColor = get(frameHandle(kHandle), 'Color');
        set(frameHandle(kHandle), 'Color', handleColor/norm(handleColor)*colorNorms(k));
        
        scale = 1 + (k-1)/10;
        
        udata = get(frameHandle(kHandle),'UData');
        set(frameHandle(kHandle),'UData', udata/scale);
        
        vdata = get(frameHandle(kHandle),'VData');
        set(frameHandle(kHandle),'VData', vdata/scale);
        
        wdata = get(frameHandle(kHandle),'WData');
        set(frameHandle(kHandle),'WData', wdata/scale);
    end
    
    % Associate a transformation to each rotation set
    handles{k} = frameHandle;
    transforms{k} = hgtransform('Parent', axHandle);
    set(handles{k}, 'Parent', transforms{k});
    
end

% Display the angular velocity pseudovector and annotate it
quiver3(0, 0, 0, w_vec(1), w_vec(2), w_vec(3), 'k', 'LineWidth', 2);
addLabelToVector(axHandle, w_vec, '{}_I \omega_{IB}');

% Setup the transforms for the coordinate systems
tInterial = hgtransform('Parent', axHandle);
set(inertialFrameHandle,'Parent',tInterial);
tBody = hgtransform('Parent', axHandle);
set(bodyHandle,'Parent',tBody);

% Set the axis limits
xlim(axHandle, limitsX);
ylim(axHandle, limitsY);
zlim(axHandle, limitsZ);

% Initial orientation using ZYX Euler angles. Angles are stored in z-y-x
% order
angles = [0;0;0];

for k=1:(t_f*updateFrequency)
    % Start a timer
    tic;
    
    dt = 1/updateFrequency;
    
    % Construct the current rotation vector
    for g=1:traceMemory
        tk = (k - (skipSteps*g-skipSteps)) / updateFrequency;
        
        % Map the rotation vector to the rotation matrix in SO(3)
        E = getMapEulAngZYXDiffToAngVelInWorldFrame(angles);
        if (rank(E) < 3 || abs(det(E)) < 0.1)
            disp('Mapping is singular!');
            disp(['Current angles: ' num2str(angles(1)) ' ' num2str(angles(2)) ' ' num2str(angles(3))]);
            %break;
        end
        anglesDot = E\I_w_IB;
        %angles = angles + dt*anglesDot;
        angles = tk*anglesDot;
        C_IB{g,1} = mapEulerAnglesZYXToRotationMatrix(angles);
        
        
        % todo: rotation matrix check sometimes fails
        %[isRot, errMsg] = isRotationMatrix(C_IB{g,1});
        %errMsg = [errMsg ' Time step: ' num2str(k)];
        %assert(isRot, errMsg);
        
        % Visualize the rotation vector
        try 
            set(transforms{g}, 'Matrix', [C_IB{g,1} zeros(3,1); zeros(1,3) 1]);
        catch
            break;
        end

    end

    % Rotate the rigid body
    try
        set(tBody, 'Matrix', [C_IB{1,1} zeros(3,1); zeros(1,3) 1]);
    catch
        break;
    end
    
    try
        drawnow;
    catch
        break;
    end
    
    % Try to synchronize the update loop
    waitTime = dt - toc;
    if (waitTime > 0)
        pause(dt-toc);
    end
    
end

end
