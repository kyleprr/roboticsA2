close all
clear
clc
set(0,'DefaultFigureWindowStyle','docked')

base = [-0.1,0.25,0];

%%
dobot = Dobot('dobot', base);
hold on

[X,Y] = meshgrid(-2:0.8:2, -2:0.8:2); % floor colour
Z = repmat(-1, size(X,1), size(X,2));
surf(X,Y,Z);

Environment.Robot_Environment()

Environment.Randomise_Tomato()
camlight

%% Camera 
q0 = deg2rad([0 -85 45 40 0]);
dobot.model.plot(q0);

cam = CentralCamera('focal',0.08, 'resolution', [1024 1024], 'centre', [512 512], 'pixel', 10e-5, 'name', 'mycamera') 


endEffector = dobot.model.fkine(q0);
endEffector(1,4) = endEffector(1,4)+0.01;
endEffector(3,4) = endEffector(3,4)+0.03;



cam.plot_camera('Tcam', endEffector, 'scale',0.02);  % place camera on end effector
cam.plot('Tcam', endEffector)
%cam.figure(1)

cam.hold(true)

%P = cam.plot(p, 'Tcam', endEffector,'o');

e = (pStar - P);
%%
J = cam.visjac_p(P, depth);

v =  lambda * pinv(J) * e(:);

Jr = dobot.model.jacobn(q0);



qVelocity = pinv(Jr)*v