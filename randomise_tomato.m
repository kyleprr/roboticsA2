
workspace = [-2 2 -2 2 -1 1];

% Generate random points
x=rand(1,3)*0.5;
y=rand(1,3)*0.2;
%scatter(x,y)

ripelocation = [x(1), y(1)];
rawlocation = [x(2), y(2)];
rottenlocation = [x(3), y(3)];

%Source (PuttingSimulatedObjectsIntoTheEnvironment):
[f,v,data] = plyread('Tomato.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
trisurf(f,v(:,1)+x(1),v(:,2)+y(1), v(:,3)+0.033, 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


[f,v,data] = plyread('Tomato.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
trisurf(f,v(:,1)+x(2),v(:,2)+y(2), v(:,3)+0.032, 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


[f,v,data] = plyread('Tomato.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
trisurf(f,v(:,1)+x(3),v(:,2)+y(3), v(:,3)+0.032, 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
