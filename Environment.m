classdef Environment
    
    methods (Static)
    
    function Robot_Environment()
    
[fd,vd,data] = plyread('Table.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+0.1,vd(:,2)+0.25,vd(:,3)-1.095, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Barrier.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)-1,vd(:,2)+2,vd(:,3)-1, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Barrier.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+1,vd(:,2)+2,vd(:,3)-1, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Barrier.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)-1,vd(:,2)-2,vd(:,3)-1, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Barrier.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+1,vd(:,2)-2,vd(:,3)-1, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');


[fd,vd,data] = plyread('Conveyer.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+0.2,vd(:,2),vd(:,3), 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

    end

    function Randomise_Tomato()
    
% Generate random points
x=rand(1,3)*0.2;
y=rand(1,3)*0.5;
%scatter(x,y)

%ripelocation = [x(1), y(1)];
%rawlocation = [x(2), y(2)];
%rottenlocation = [x(3), y(3)];



%Source (PuttingSimulatedObjectsIntoTheEnvironment):
[f,v,data] = plyread('Tomato_Ripe.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
trisurf(f,v(:,1)+x(1),v(:,2)+y(1), v(:,3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


[f,v,data] = plyread('Tomato_Raw.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
trisurf(f,v(:,1)+x(2),v(:,2)+y(2), v(:,3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


[f,v,data] = plyread('Tomato_Rotten.ply','tri');
% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
trisurf(f,v(:,1)+x(3),v(:,2)+y(3), v(:,3), 'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat'); 
        
    end
    
    
    
    end

end