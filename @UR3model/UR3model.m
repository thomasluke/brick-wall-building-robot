%% UR3 Model
%  41013 Robotics
%  Thomas Harrison
%  August 2020

classdef UR3model < handle
    properties
        
        model
        
        % Set the size of the workspace when drawing the robot
        workspace = [-2 2 -2 2 -0.3 2];
        
        % Flag to indicate if gripper is used
        useGripper = false;
    
    end
    
    methods
        
function self = UR3model(useGripper)
    if nargin < 1
        useGripper = false;
    end
    self.useGripper = useGripper;
    

self.GetUR3Robot();

self.PlotAndColourRobot();

end

%% GetUR3Robot
% Given a name (optional), create and return a UR3 robot model
function GetUR3Robot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['UR_3_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end

L(1) = Link([pi 0.1519 0 pi/2]);

L(2) = Link([0 0 -0.24365 0]);

L(3) = Link([0 0 -0.21325 0]);

L(4) = Link([0 0.11235 0 pi/2]);

L(5) = Link([0 0.08535 0 -pi/2]);

L(6) = Link([0 0.0819 0 0]);

 % Incorporate joint limits
    L(1).qlim = [-360 360]*pi/180;
    L(2).qlim = [-90 90]*pi/180;
    L(3).qlim = [-170 170]*pi/180;
    L(4).qlim = [-360 360]*pi/180;
    L(5).qlim = [-360 360]*pi/180;
    L(6).qlim = [-360 360]*pi/180;

    L(2).offset = -pi/2;
    L(4).offset = -pi/2;
    
    self.model = SerialLink(L,'name',name);

end

%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['UR3Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
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
