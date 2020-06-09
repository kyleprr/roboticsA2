classdef Environment
    
    methods (Static)
    
    function Robot_Environment()
    
     greenTray = [0.2 0.6 0];
     brownTray = [0 0.6 0];
     Estop = [0.9 0.6 0];
        
[fd,vd,data] = plyread('Table.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+0.1,vd(:,2)+0.25,vd(:,3)-0.876, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Barrier.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1),vd(:,2)+1.2,vd(:,3)-0.876, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Barrier.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1),vd(:,2)-1.2,vd(:,3)-0.876, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Barrier1.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)-1.2,vd(:,2),vd(:,3)-0.876, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Barrier1.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+1.2,vd(:,2),vd(:,3)-0.876, 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');


[fd,vd,data] = plyread('Conveyer.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+0.15,vd(:,2),vd(:,3), 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');

[fd,vd,data] = plyread('Trayg.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+greenTray(1),vd(:,2)+greenTray(2),vd(:,3), 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');


[fd,vd,data] = plyread('Trayb.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+brownTray(1),vd(:,2)+brownTray(2),vd(:,3), 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');


[fd,vd,data] = plyread('EStop.ply', 'tri');
vertexcolours = [data.vertex.red, data.vertex.green, data.vertex.blue]/255;    %colour scaled down to 1
trisurf(fd, vd(:,1)+Estop(1),vd(:,2)+Estop(2),vd(:,3), 'FaceVertexCData', vertexcolours, ...
    'EdgeLighting', 'flat');
    end


    
    
    end

end