function animateLinearVelocity()
% ANIMATELINEARVELOCITY() 
% generates an animation showing the rotation of a coordinate system w.r.t.
% a fixed one.
%
% Author(s): Dario Bellicoso

% Provide a default scaling for the angular velocity
if nargin < 2
    rate = 1;
end

% Provide a default redraw frequency
if nargin < 3
    updateFrequency = 50; % Hz
end

% Setup figure
limit = 2.0;
limitsX = [-limit limit];
limitsY = [-limit limit];
limitsZ = [-limit limit];
figure();
axHandle = axes();
%view(3);
hold on; grid on; axis equal;

% Set the axis limits
xlim(axHandle, limitsX);
ylim(axHandle, limitsY);
zlim(axHandle, limitsZ);

% Animate for t_f seconds
t_f = 90.0;

% Visualize the coordinate frame unit vectors and get handles to them
[axHandle, iFrameHandle] = visualizeCoordinateSystem(eye(3), axHandle, 'I');
[axHandle, bFrameHandle] = visualizeCoordinateSystem(eye(3), axHandle, 'B');

% Draw an ellipsoid as a rigid body representation
bodyScale = 0.05;
bodyDim = [1.0; 1.0; 1.0];
[x, y, z] = ellipsoid(0, 0, 0, bodyScale*bodyDim(1), bodyScale*bodyDim(2), bodyScale*bodyDim(3), 50);
bodyHandle = surf(x, y, z, 2*z);
set(bodyHandle, 'edgecolor','none');

% Setup the transforms for the coordinate systems
tIFrame = hgtransform('Parent', axHandle);
tBFrame = hgtransform('Parent', axHandle);
tBody   = hgtransform('Parent', axHandle);
set(iFrameHandle,'Parent',tIFrame);
set(bFrameHandle,'Parent',tBFrame);
set(bodyHandle,'Parent',tBody);

% Initial pose of frame B
I_r_IB = rand(3,1);
ph_IB = [0; 0; rand(1,1)];
C_IB = mapRotationVectorToRotationMatrix(ph_IB);
T_IB = [C_IB I_r_IB; zeros(1,3) 1];
set(tBFrame, 'Matrix', T_IB);

% Body motion settings
sineFreq = 0.1; % sine frequency [Hz]
sineA = 0.4; % sine amplitude
B_r_BP = sineA*[cos(0); sin(0); 0];
I_r_IP = I_r_IB + C_IB*(B_r_BP);
B_r_IP = C_IB'*I_r_IP;

bodyVelocityInInertialFrameHandle = quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 1);
bodyVelocityInMovingFrameHandle = quiver3(0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 1);
bFrameVelocityHandle = quiver3(0, 0, 0, 0, 0, 0, 'm', 'LineWidth', 1);
angVel = 0.5;

% Create transforms for trace.
traceLength = 70;
pointTransformsBodyFrame = cell(traceLength,1);
pointTransformsInertialFrame = cell(traceLength,1);
pointTransformsMovingFrame = cell(traceLength,1);
pointHistoryBodyFrame = cell(traceLength,1);
pointHistoryInertialFrame = cell(traceLength,1);
pointHistoryMovingFrame = cell(traceLength,1);
for k=1:traceLength
    pointHandleBodyFrame = plot3(0, 0, 0, 'b.');
    pointHandleInertialFrame = plot3(0, 0, 0, 'r.');
    pointHandleMovingFrame = plot3(0, 0, 0, 'k.');
    
    pointTransformsBodyFrame{k} = hgtransform('Parent', axHandle);
    pointTransformsInertialFrame{k} = hgtransform('Parent', axHandle);
    pointTransformsMovingFrame{k} = hgtransform('Parent', axHandle);
    
    set(pointHandleBodyFrame,'Parent',pointTransformsBodyFrame{k});
    set(pointHandleInertialFrame,'Parent',pointTransformsInertialFrame{k});
    set(pointHandleMovingFrame,'Parent',pointTransformsMovingFrame{k});
    
    pointHistoryBodyFrame{k} = [0;0;0];
    pointHistoryInertialFrame{k} = [0;0;0];
    pointHistoryMovingFrame{k} = [0;0;0];
end

skipUpdates = 4;
counter = 1;

dt = 1/updateFrequency;
currentInterTime = dt;
for k=1:(t_f*updateFrequency)
    tic;
    if (k~=1)
        %currentInterTime = toc(loopStartTime);
    end
    
    % Start a timer.
    loopStartTime = tic;
    
    % Set angular motion of frame B.
    sineParam = 2*pi*sineFreq*k*currentInterTime;
    I_w_IB = [0; 0; angVel];
    ph_IB = ph_IB + I_w_IB*currentInterTime;
    C_IB = mapRotationVectorToRotationMatrix(ph_IB);
    %C_IB = expm(currentInterTime*skew(I_w_IB));
    
    % Set linear motion of frame B.
    I_v_IB = 2*pi*sineFreq*sineA*[-sin(sineParam); cos(sineParam); 0];
    I_r_IB = I_r_IB + I_v_IB*currentInterTime;
    
    % Set motion of the body.
    B_dr_BP = 2*pi*sineFreq*sineA*[-sin(sineParam); cos(sineParam); 0];
    B_r_BP  = B_r_BP + B_dr_BP*currentInterTime;
    I_r_BP  = C_IB*B_r_BP;
    
    % Integrate motion.
    I_v_IP = I_v_IB + C_IB*B_dr_BP + cross(I_w_IB, I_r_BP);
    %I_r_IP = I_r_IP + I_v_IP/updateFrequency;
    
    % Test: get motion from a moving frame.
    B_w_IB = C_IB'*I_w_IB;
    B_w_BI = -B_w_IB;
    
    
    
    B_v_IP = C_IB'*I_v_IP + cross(B_w_BI, B_r_IP);
    %B_v_IP = C_IB'*I_v_IP + cross(B_w_IB, B_r_IP);
    
    
    
    
    
    B_r_IP = B_r_IP + B_v_IP*currentInterTime;
    I_r_IP = C_IB*B_r_IP;
    
    I_v_BP = C_IB*B_dr_BP;
    
    % Update transforms.
    try
        %plot3(I_r_IP(1), I_r_IP(2), I_r_IP(3), 'b.');
        set(bodyVelocityInInertialFrameHandle, 'XData', I_r_IP(1));
        set(bodyVelocityInInertialFrameHandle, 'YData', I_r_IP(2));
        set(bodyVelocityInInertialFrameHandle, 'ZData', I_r_IP(3));
        set(bodyVelocityInInertialFrameHandle, 'UData', I_v_IP(1));
        set(bodyVelocityInInertialFrameHandle, 'VData', I_v_IP(2));
        set(bodyVelocityInInertialFrameHandle, 'WData', I_v_IP(3));
        
        set(bodyVelocityInMovingFrameHandle, 'XData', I_r_IP(1));
        set(bodyVelocityInMovingFrameHandle, 'YData', I_r_IP(2));
        set(bodyVelocityInMovingFrameHandle, 'ZData', I_r_IP(3));
        set(bodyVelocityInMovingFrameHandle, 'UData', I_v_BP(1));
        set(bodyVelocityInMovingFrameHandle, 'VData', I_v_BP(2));
        set(bodyVelocityInMovingFrameHandle, 'WData', I_v_BP(3));
    
        T_IP = [C_IB I_r_IP; zeros(1,3) 1];
        set(tBody, 'Matrix', T_IP);
        
        T_IB = [C_IB I_r_IB; zeros(1,3) 1];
        set(tBFrame, 'Matrix', T_IB);
        
        if (mod(k,skipUpdates) == 0)
            idx = mod(counter, traceLength) + 1;
            pointHistoryBodyFrame{idx} = B_r_BP;
            pointHistoryInertialFrame{idx} = I_r_IP;
            pointHistoryMovingFrame{idx} = I_r_IB;
            counter = counter + 1;
        end
        
        for kk=1:traceLength
            T_BP = [C_IB I_r_IB + C_IB*pointHistoryBodyFrame{kk}; zeros(1,3) 1];
            set(pointTransformsBodyFrame{kk}, 'Matrix', T_BP);
            
            T_IP = [C_IB pointHistoryInertialFrame{kk}; zeros(1,3) 1];
            set(pointTransformsInertialFrame{kk}, 'Matrix', T_IP);
            
            T_IB = [C_IB pointHistoryMovingFrame{kk}; zeros(1,3) 1];
            set(pointTransformsMovingFrame{kk}, 'Matrix', T_IB);
        end
        
    catch
        disp('error while updating');
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
