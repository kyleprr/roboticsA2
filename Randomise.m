% Code to randomise Tomatoes on the conveyer belt:

% PuttingSimulatedObjectsIntoTheEnvironment.m:

[f,v,data] = plyread('Tomato-Test.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

keyboard
% clf



% Using Lab 2 Exercise files:
% 2.3 Create an instance of the cow herd with default parameters
cowHerd = RobotCows();
% 2.4 Check how many cow there are
cowHerd.cowCount
% 2.5 Plot on single iteration of the random step movement
cowHerd.PlotSingleRandomStep



% 2.7 Test many random steps
numSteps=100;
delay=0.01;
cowHerd.TestPlotManyStep(numSteps,delay);