classdef Dobot < handle
    
   % Sourced from UR3 Class and Peter Crke's toolbox robot models (UR5)
     properties 
         model;
         
         workspace = [-2 2 -2 2 -1 1];
    
     end 
     
     
     methods 
            
        function self = Dobot(name, base_location)
            
            %Create UR3 robot
            GetDobot(self, name, base_location);
            % Load and Plot Robot Links
            PlotAndColourRobot(self);
            drawnow();
               
                drawnow();
            end
        
    
                %% GetUR3Robot
        % Create and return a UR3 robot model
        function GetDobot(self, name, base_location)
            
L1 = Link('d',0.138,'a',0,'alpha',-pi/2,'offset', pi, 'qlim',[-2*pi 2*pi]);
L2 = Link('d',0,'a',0.135,'alpha',0,'offset', 0, 'qlim',deg2rad([-70 25]));
L3 = Link('d',0,'a',0.147,'alpha',0,'offset', 0, 'qlim',deg2rad([-40 120])) ;  
L4 = Link('d', 0, 'a', 0.055, 'alpha', -pi/2, 'offset', 0, 'qlim', deg2rad([-70 400]));
L5 = Link('d', 0.055, 'a', 0, 'alpha', 0, 'offset', 0,  'qlim', [0 0]);

            
            self.model = SerialLink([L1 L2 L3 L4 L5],'name',name);
            self.model.base = transl(base_location);
                
        end             
                %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'workspace',self.workspace);
            self.model.plotopt = {'nojoints','noname', 'noshadow','nobase', 'nowrist','notiles','scale',0}; %

            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end   
        
        
     
        
     end 
        
   
        
end
