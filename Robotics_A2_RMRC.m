close all
clear
clc
set(0,'DefaultFigureWindowStyle','docked')

baseUR3 = [0.4,0.25,0];
baseDobot = [0.6 -0.15 0];


basketRed = [-0.1 0 0];
basketGreen = [-0.1 0.5 0];
basketBrown = [-0.25 0.15 0];

% Generate random tomato locations
xt=rand(1,3)*0.16;
yt=rand(1,3)*0.4;

trayRipe = [0.64 0.087 0]; %tray location for dobot to pick up
trayRipeUR3 = [0.3 -0.15 0]; % tray location for UR3 
trayRaw = [0.2 0.6 0];
trayRotten = [0 0.6 0];

cube = [-0.5 0.2 0];% for collision detection x = 0.1

%%
robot = UR3('robot', baseUR3);
hold on

dobot = Dobot('dobot', baseDobot); dobot.model.base = dobot.model.base*trotz(pi);

[X,Y] = meshgrid(-2:0.8:2, -2:0.8:2);   % floor colour
Z = repmat(-0.88, size(X,1), size(X,2));
surf(X,Y,Z);

Environment.Robot_Environment()         % setup environment 
camlight                                % lights, camera, action
view(3)
%% Tomatos

qr = deg2rad([0 -60 60 -90 -90 0]); robot.model.plot(qr); % UR3 intial pose
qd = deg2rad([90 -60 80 -20 0]);    dobot.model.plot(qd); % Dobot intial pose

%Source (PuttingSimulatedObjectsIntoTheEnvironment): https://au.mathworks.com/matlabcentral/fileexchange/58774-putting-simulated-objects-into-the-environment

[f,v,data] = plyread('Tomato_Ripe.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;  % Scale the colours to be 0-to-1 (they are originally 0-to-255
ripe = trisurf(f,v(:,1)+xt(1),v(:,2)+yt(1), v(:,3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

[f,v,data] = plyread('Tomato_Raw.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;  % Scale the colours to be 0-to-1 (they are originally 0-to-255
raw = trisurf(f,v(:,1)+xt(2),v(:,2)+yt(2), v(:,3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

[f,v,data] = plyread('Tomato_Rotten.ply','tri');       
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;  % Scale the colours to be 0-to-1 (they are originally 0-to-255
rotten = trisurf(f,v(:,1)+xt(3),v(:,2)+yt(3), v(:,3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat'); 

[f,v,data] = plyread('Trayr.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;  % Scale the colours to be 0-to-1 (they are originally 0-to-255
tray = trisurf(f,v(:,1)+trayRipe(1),v(:,2)+trayRipe(2), v(:,3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat'); 

[fc,vc,datac] = plyread('Cube.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [datac.vertex.red, datac.vertex.green, datac.vertex.blue] / 255;  % Scale the colours to be 0-to-1 (they are originally 0-to-255
trisurf(fc,vc(:,1)+cube(1),vc(:,2)+cube(2), vc(:,3)+cube(3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

cubeFaceNormals = [datac.vertex.nx,datac.vertex.ny,datac.vertex.nz];


%% Dobtot Motion
steps = 30;
s = lspb(0,1,steps);           %Trapezoidal method - dobot joint values matrix

    dMatrix = nan(steps,5);        % dMatrix size 30x5

    qRipe = dobot.model.ikcon(transl(trayRipe), qd);
    qt = deg2rad([80 -21 86 -65 0]);                           % [0.64 0.087 0] tray location for dobot to pick up 

 for i = 1:steps                                               %Loop to go from dobot's current position to tray location (qt)                                               
    dMatrix(i,:) = (1-s(i))*dobot.model.getpos + s(i)*qt;      %Set of Joint angles to go from q0 to qt.
    dobot.model.plot(dMatrix(i,:),'fps',60)                   
 end    

    qTrayLocation = deg2rad([180 -6 50 -50 0]);                % joint configuration at next tray location (near UR3)
    wayPoint = deg2rad([150 -20 60 -45 0]);                    % waypoint

%Loop to go from tray to next waypoint 
 for i = 1:steps                                                                                          
    dMatrix(i,:) = (1-s(i))*qt + s(i)*wayPoint;                %Set of Joint angles to go from qt to waypoint location.
    dobot.model.plot(dMatrix(i,:),'fps',60)                    %Plot rows of joint angles with robot2  
    try 
        delete(tray);
    end
    
    dobotEE = dobot.model.fkine(dMatrix(i,:));                  
    
    [f,v,data] = plyread('Trayr.ply','tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    tray = trisurf(f,v(:,1)+dobotEE(1,4),v(:,2)+dobotEE(2,4), v(:,3)+dobotEE(3,4), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
 end    


 for i = 1:steps                                            % loop to go from waypoint to tray location                                                                                          
    dMatrix(i,:) = (1-s(i))*wayPoint + s(i)*qTrayLocation;  % Set of Joint angles to go from qi to qf.
    dobot.model.plot(dMatrix(i,:),'fps',60)                 % Plot rows of joint angles with robot2  
        try 
        delete(tray);
    end
    
    dobotEE = dobot.model.fkine(dMatrix(i,:));                  
    
    [f,v,data] = plyread('Trayr.ply','tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    tray = trisurf(f,v(:,1)+dobotEE(1,4),v(:,2)+dobotEE(2,4), v(:,3)+dobotEE(3,4), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
    
    
 end    

 
 for i = 1:steps                                           % dobot return to home location                                                
    dMatrix(i,:) = (1-s(i))*dobot.model.getpos + s(i)*qt;  % Set of Joint angles to go from qi to qf.
    dobot.model.plot(dMatrix(i,:),'fps',60)                    
 end    
    
%% RMRC - Going to red tomato UR3  
% RMRC code sourced from Lab9 exercise, code slightly optimised to save lines 
%https://online.uts.edu.au/bbcswebdav/pid-3780404-dt-content-rid-58309711_1/courses/41013-2020-AUTUMN-CITY/Lab%209%20Exercises.pdf

urEE = robot.model.fkine(qr);   % endEffector location
urEE = urEE(1:3,4);      % endEffector translational component

t = 10;                                % Total time (s)
deltaT = 0.5;                          % Control frequency
steps = t/deltaT;                      % No. of steps for simulation
delta = (deg2rad(20))/steps;           % Small angle change
epsilon = 0.1;                         % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);         % Weighting matrix for the velocity vector

% 1.3) Set up trajectory, initial pose
s = lspb(0,1,steps);                   % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*urEE(1) + s(i)*xt(1,1);    % Points in x
    x(2,i) = (1-s(i))*urEE(2) + s(i)*yt(1,1);    % Points in y
    x(3,i) = (1-s(i))*urEE(3) + s(i)*0.02;       % Points in z
    theta(1,i) = 0; theta(2,i) = 0; theta(3,i) = 0;     % Roll, Pitch, Yaw angles 
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];       % Create transformation of first point and angle

qMatrix(1,:) = robot.model.ikcon(T*trotx(pi),robot.model.getpos());      % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                            % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                         % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                             % Calculate rotation matrix error
    S = Rdot*Ra';                                            % Skew symmetric!
    deltaTheta = tr2rpy(Rd*Ra');                             % Convert rotation matrix to RPY angles
    xdot = W*[(1/deltaT)*deltaX;[S(3,2);S(1,3);S(2,1)]];     % end-effector velocity to reach next waypoint. (translation & rotation velocities combined
    J = robot.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
    J = J(1:6,1:6);
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                        % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                           % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                         % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)      % If next joint angle is lower than joint limit... stop robot
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)  % If next joint angle is greater than joint limit ... stop robot
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                  	% Update next joint state based on joint velocities

end

for i = 1:steps
% IsCollision(robot.model,qMatrix(i,:),fc,vc,cubeFaceNormals,false);  

robot.model.plot(qMatrix(i,:));  % Plot the trajectories 
end
 clear qMatrix; clear x; clear theta;    % clear variables to be reused

%% Red tomato to basket

urEE = robot.model.fkine(robot.model.getpos());   % endEffector location
urEE = urEE(1:3,4);                               % endEffector translational component

s = lspb(0,1,steps);                              % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*urEE(1) + s(i)*trayRipeUR3(1);         % Points in x
    x(2,i) = (1-s(i))*urEE(2) + s(i)*trayRipeUR3(2);         % Points in y
    x(3,i) = (1-s(i))*urEE(3) + s(i)*trayRipeUR3(3)*0.05+0.01;    % Points in z
    theta(1,i) = 0; theta(2,i) = 0; theta(3,i) = 0;          % Roll, Pitch, Yaw angles 
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];       % Create transformation of first point and angle

qMatrix(1,:) = robot.model.ikcon(T*trotx(pi),robot.model.getpos());      % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                            % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                         % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                             % Calculate rotation matrix error
    S = Rdot*Ra';                                            % Skew symmetric!
    deltaTheta = tr2rpy(Rd*Ra');                             % Convert rotation matrix to RPY angles
    xdot = W*[(1/deltaT)*deltaX;[S(3,2);S(1,3);S(2,1)]];     % end-effector velocity to reach next waypoint. (translation & rotation velocities combined
    J = robot.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
    J = J(1:6,1:6);
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                        % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                           % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                         % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)      % If next joint angle is lower than joint limit... stop robot
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)  % If next joint angle is greater than joint limit ... stop robot
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                  	% Update next joint state based on joint velocities

end

%robot.model.plot(qMatrix);  % Plot the trajectories 

for i = 1:steps                                              %                                              
     robot.model.plot(qMatrix(i,:),'fps',60)                
% IsCollision(robot.model,qMatrix,fc,vc,cubeFaceNormals,false); % Colliion check
     
    try 
        delete(ripe);
    end
    
    urEE = robot.model.fkine(qMatrix(i,:));                  
    
    [f,v,data] = plyread('Tomato_Ripe.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
ripe = trisurf(f,v(:,1)+urEE(1,4),v(:,2)+urEE(2,4), v(:,3)+urEE(3,4)-0.028, 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

end

try
    delete(ripe)
end 
   [f,v,data] = plyread('Tomato_Ripe.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
ripe = trisurf(f,v(:,1)+trayRipeUR3(1),v(:,2)+trayRipeUR3(2), v(:,3)+trayRipeUR3(3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

 clear qMatrix; clear x; clear theta;    % clear variables to be reused

%% Box to green tomato

urEE = robot.model.fkine(robot.model.getpos());   % endEffector location
urEE = urEE(1:3,4);                               % endEffector translational component

s = lspb(0,1,steps);                              % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*urEE(1) + s(i)*xt(2);       % Points in x
    x(2,i) = (1-s(i))*urEE(2) + s(i)*yt(2);       % Points in y
    x(3,i) = (1-s(i))*urEE(3) + s(i)*0.03+0.01;   % Points in z
    theta(1,i) = 0; theta(2,i) = 0; theta(3,i) = 0;     % Roll, Pitch, Yaw angles 
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];       % Create transformation of first point and angle

qMatrix(1,:) = robot.model.ikcon(T*trotx(pi),robot.model.getpos());      % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                            % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                         % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                             % Calculate rotation matrix error
    S = Rdot*Ra';                                            % Skew symmetric!
    deltaTheta = tr2rpy(Rd*Ra');                             % Convert rotation matrix to RPY angles
    xdot = W*[(1/deltaT)*deltaX;[S(3,2);S(1,3);S(2,1)]];     % end-effector velocity to reach next waypoint. (translation & rotation velocities combined
    J = robot.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
    J = J(1:6,1:6);
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                        % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                           % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                         % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)      % If next joint angle is lower than joint limit... stop robot
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)  % If next joint angle is greater than joint limit ... stop robot
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                  	% Update next joint state based on joint velocities

end
% IsCollision(robot.model,qMatrix,fc,vc,cubeFaceNormals,false);         % Collision check deactivated 
robot.model.plot(qMatrix);  % Plot the trajectories 

clear qMatrix; clear x; clear theta;                                    % clear variables to be reused

%%  Green tomato to box

urEE = robot.model.fkine(robot.model.getpos());   % endEffector location
urEE = urEE(1:3,4);                               % endEffector translational component

s = lspb(0,1,steps);                              % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*urEE(1) + s(i)*trayRaw(1);        % Points in x
    x(2,i) = (1-s(i))*urEE(2) + s(i)*trayRaw(2);        % Points in y
    x(3,i) = (1-s(i))*urEE(3) + s(i)*trayRaw(3)*0.03+0.01;    % Points in z
    theta(1,i) = 0; theta(2,i) = 0; theta(3,i) = 0;     % Roll, Pitch, Yaw angles 
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];       % Create transformation of first point and angle

qMatrix(1,:) = robot.model.ikcon(T*trotx(pi),robot.model.getpos());      % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                            % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                         % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                             % Calculate rotation matrix error
    S = Rdot*Ra';                                            % Skew symmetric!
    deltaTheta = tr2rpy(Rd*Ra');                             % Convert rotation matrix to RPY angles
    xdot = W*[(1/deltaT)*deltaX;[S(3,2);S(1,3);S(2,1)]];     % end-effector velocity to reach next waypoint. (translation & rotation velocities combined
    J = robot.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
    J = J(1:6,1:6);
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                        % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                           % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                         % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)      % If next joint angle is lower than joint limit... stop robot
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)  % If next joint angle is greater than joint limit ... stop robot
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                  	% Update next joint state based on joint velocities

end


for i = 1:steps                                                         % Loop for carrying green tomato                                 
    robot.model.plot(qMatrix(i,:),'fps',60)                
    % IsCollision(robot.model,qMatrix,fc,vc,cubeFaceNormals,false);

    try 
        delete(raw);
    end
    
    urEE = robot.model.fkine(qMatrix(i,:));                  
    
    [f,v,data] = plyread('Tomato_Raw.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
raw = trisurf(f,v(:,1)+urEE(1,4),v(:,2)+urEE(2,4), v(:,3)+urEE(3,4)-0.028, 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

end

try
    delete(raw)
end 
   [f,v,data] = plyread('Tomato_Raw.ply','tri');
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
raw = trisurf(f,v(:,1)+trayRaw(1),v(:,2)+trayRaw(2), v(:,3)+trayRaw(3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

 clear qMatrix; clear x; clear theta;    % clear variables to be reused
 
 %% Box to brown tomato
 
urEE = robot.model.fkine(robot.model.getpos());   % endEffector location
urEE = urEE(1:3,4);                               % endEffector translational component

s = lspb(0,1,steps);                              % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*urEE(1) + s(i)*xt(3);       % Points in x
    x(2,i) = (1-s(i))*urEE(2) + s(i)*yt(3);       % Points in y
    x(3,i) = (1-s(i))*urEE(3) + s(i)*0.03+0.01;   % Points in z
    theta(1,i) = 0; theta(2,i) = 0; theta(3,i) = 0;     % Roll, Pitch, Yaw angles 
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];       % Create transformation of first point and angle

qMatrix(1,:) = robot.model.ikcon(T*trotx(pi),robot.model.getpos());      % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                            % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                         % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                             % Calculate rotation matrix error
    S = Rdot*Ra';                                            % Skew symmetric!
    deltaTheta = tr2rpy(Rd*Ra');                             % Convert rotation matrix to RPY angles
    xdot = W*[(1/deltaT)*deltaX;[S(3,2);S(1,3);S(2,1)]];     % end-effector velocity to reach next waypoint. (translation & rotation velocities combined
    J = robot.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
    J = J(1:6,1:6);
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                        % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                           % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                         % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)      % If next joint angle is lower than joint limit... stop robot
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)  % If next joint angle is greater than joint limit ... stop robot
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                  	% Update next joint state based on joint velocities

end

% IsCollision(robot.model,qMatrix,fc,vc,cubeFaceNormals);               % Collision check deactivated

robot.model.plot(qMatrix);  % Plot the trajectories 

 clear qMatrix; clear x; clear theta;    % clear variables to be reused
 
 %% Brown tomato to box
 
urEE = robot.model.fkine(robot.model.getpos());   % endEffector location
urEE = urEE(1:3,4);                               % endEffector translational component

s = lspb(0,1,steps);                              % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*urEE(1) + s(i)*trayRotten(1);         % Points in x
    x(2,i) = (1-s(i))*urEE(2) + s(i)*trayRotten(2);         % Points in y
    x(3,i) = (1-s(i))*urEE(3) + s(i)*trayRotten(3)*0.03+0.01;    % Points in z
    theta(1,i) = 0; theta(2,i) = 0; theta(3,i) = 0;         % Roll, Pitch, Yaw angles 
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];       % Create transformation of first point and angle

qMatrix(1,:) = robot.model.ikcon(T*trotx(pi),robot.model.getpos());      % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                            % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                         % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                             % Calculate rotation matrix error
    S = Rdot*Ra';                                            % Skew symmetric!
    deltaTheta = tr2rpy(Rd*Ra');                             % Convert rotation matrix to RPY angles
    xdot = W*[(1/deltaT)*deltaX;[S(3,2);S(1,3);S(2,1)]];     % end-effector velocity to reach next waypoint. (translation & rotation velocities combined
    J = robot.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
    J = J(1:6,1:6);
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                        % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                           % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                         % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)      % If next joint angle is lower than joint limit... stop robot
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)  % If next joint angle is greater than joint limit ... stop robot
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                  	% Update next joint state based on joint velocities

end


for i = 1:steps                                                    
    robot.model.plot(qMatrix(i,:),'fps',60)                
% IsCollision(robot.model,qMatrix,fc,vc,cubeFaceNormals);   Collision check deactivated
    
    try 
        delete(rotten);
    end
    
    urEE = robot.model.fkine(qMatrix(i,:));                  
    
    [f,v,data] = plyread('Tomato_Rotten.ply','tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    rotten = trisurf(f,v(:,1)+urEE(1,4),v(:,2)+urEE(2,4), v(:,3)+urEE(3,4)-0.028, 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

end

try
    delete(rotten)
end 
    [f,v,data] = plyread('Tomato_Rotten.ply','tri');
    vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
    rotten = trisurf(f,v(:,1)+trayRotten(1),v(:,2)+trayRotten(2), v(:,3)+trayRotten(3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

%% Robot retreat

    urRetreat = robot.model.fkine(qr);                  % origin configuration
    urEE = robot.model.fkine(robot.model.getpos());     % endEffector location
    urEE = urEE(1:3,4);                                 % endEffector translational component
for i=1:steps
    x(1,i) = (1-s(i))*urEE(1) + s(i)*urRetreat(1,4);    % Points in x
    x(2,i) = (1-s(i))*urEE(2) + s(i)*urRetreat(2,4);    % Points in y
    x(3,i) = (1-s(i))*urEE(3) + s(i)*urRetreat(3,4);    % Points in z
    theta(1,i) = 0; theta(2,i) = 0; theta(3,i) = 0;     % Roll, Pitch, Yaw angles 
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];       % Create transformation of first point and angle

qMatrix(1,:) = robot.model.ikcon(T*trotx(pi),robot.model.getpos());      % Solve joint angles to achieve first waypoint

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = robot.model.fkine(qMatrix(i,:));                     % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                            % Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));      % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                         % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                             % Calculate rotation matrix error
    S = Rdot*Ra';                                            % Skew symmetric!
    deltaTheta = tr2rpy(Rd*Ra');                             % Convert rotation matrix to RPY angles
    xdot = W*[(1/deltaT)*deltaX;[S(3,2);S(1,3);S(2,1)]];     % end-effector velocity to reach next waypoint. (translation & rotation velocities combined
    J = robot.model.jacob0(qMatrix(i,:));                    % Get Jacobian at current joint state
    J = J(1:6,1:6);
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                        % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(6))*J';                               % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                           % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                         % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)      % If next joint angle is lower than joint limit... stop robot
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)  % If next joint angle is greater than joint limit ... stop robot
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                  	% Update next joint state based on joint velocities

end

robot.model.plot(qMatrix);