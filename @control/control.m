classdef control < handle
    % control contains functions and parameters to control robot movements

    properties
        
    end
    
    methods
        
        function MoveEndEffectorToPoint(obj,robots,ur3Brick,ur5Brick,ur3EndPoint,ur5EndPoint)
                         
            t = [0:0.05:2];
            
            % Define number of positions for each move (higher number = slower but smoother movements)
            trajectoryLength = 100;
            
            ur3 = robots{1};
            ur5 = robots{2};
            
            % UR3 Parameters
            
            % Starting Foward kinematics for UR3
            ur3StartFkine = ur3.model.fkine(ur3.model.getpos());
            
            % Change in end effector position from the starting to desired end point
            ur3DeltaPosition = [(ur3EndPoint(1)-ur3StartFkine(1,4)),  (ur3EndPoint(2)-ur3StartFkine(2,4)), (ur3EndPoint(3)-ur3StartFkine(3,4))]; % x, y, -z
            
            % Transform starting Fkine using change in posotion to find the end Fkine
            ur3EndFkine = transl(ur3DeltaPosition)*ur3StartFkine;
            
            % Use EndFkine to finde the joint orientations for the end position
            ur3EndIkcon = ur3.model.ikcon(ur3EndFkine);
            
            % Finding the robot joint positions required to move the end effector to the end point
            ur3Traj = jtraj(ur3StartFkine,ur3EndFkine,t);
            ur3JointTrajectory = jtraj(ur3.model.getpos(),ur3EndIkcon,trajectoryLength);
            
            % UR5 Parameters
            
            ur5StartFkine = ur5.model.fkine(ur5.model.getpos());
                        
            % Including a small error of -0.001 along the x axis to stop a minor bug (similar to matlabs eps which is too small for this application) 
            ur5DeltaPosition = [(ur5EndPoint(1)-ur5StartFkine(1,4))-0.001,  (ur5EndPoint(2)-ur5StartFkine(2,4)), (ur5EndPoint(3)-ur5StartFkine(3,4))]; % -y, -z, -x 
                            
            ur5EndFkine = transl(ur5DeltaPosition)*ur5StartFkine;
            ur5EndIkcon = ur5.model.ikcon(ur5EndFkine);
            
            ur5Traj = jtraj(ur5StartFkine,ur5EndFkine,t);
            ur5JointTrajectory = jtraj(ur5.model.getpos(),ur5EndIkcon,trajectoryLength);
            
            % Iterate the robot arms through their movement
            for trajStep = 1:size(ur3JointTrajectory,1);
               
                ur3Q = ur3JointTrajectory(trajStep,:);
                ur5Q = ur5JointTrajectory(trajStep,:);
                
                ur3Fkine = ur3.model.fkine(ur3.model.getpos());
                ur5Fkine = ur5.model.fkine(ur5.model.getpos());
                
                disp('UR3 transform:')
                disp(ur3Fkine)
                
                disp('UR5 transform:')
                disp(ur5Fkine)
                
                % If robot is currently holding a brick then move the brick with the end effector
                if ur3Brick ~= 0 
                    MoveObject(ur3Brick,[ur3Fkine(1,4),ur3Fkine(2,4),ur3Fkine(3,4)],0);
                end
                
                % If robot is currently holding a brick then move the brick with the end effector
                if ur5Brick ~= 0
                    MoveObject(ur5Brick,[ur5Fkine(1,4),ur5Fkine(2,4),ur5Fkine(3,4)],0);
                end
                
                % Animate robot through a fraction of the total movement
                ur3.model.animate(ur3Q);
                ur5.model.animate(ur5Q);
                
                drawnow();
            
            end
            
        end
        
        
%         function MoveEndEffectorToPoint(obj,robots,robotBricks,robotEndPoints)
%             
%             t = [0:0.05:2];
%             
%             trajectoryLength = 100;
%             
%             numberOfRobotsArray = size(robots);
%             numberOfRobots = numberOfRobotsArray(end);
%            
%             robotStartFkine = cell(numberOfRobots) ;
%             robotDeltaPosition = cell(numberOfRobots);
%             robotEndFkine = cell(numberOfRobots);
%             robotEndIkcon = cell(numberOfRobots);
%             robotTraj = cell(numberOfRobots);
%             robotJointTrajectory = cell(numberOfRobots);
%             
%             
%             
%             for i=1:1:numberOfRobots
%                 % UR3 Parameters
%                 robot = robots{i};
%                 
%                 robotStartFkine{i} = robot.model.fkine(robot.model.getpos());
%                 % ur3StartIkcon = ur3.model.ikcon(ur3StartFkine)
%                 
%                 robotEndPoint = robotEndPoints{i};
%                 startFkine = robotStartFkine{i};
%                 
%                 robotDeltaPosition{i} = [(robotEndPoint(1)-startFkine(1,4)),  (robotEndPoint(2)-startFkine(2,4)), (robotEndPoint(3)-startFkine(3,4))]; % x, y, -z
%                 
%                 robotEndFkine{i} = transl(robotDeltaPosition{i})*robotStartFkine{i};
%                 robotEndIkcon{i} = robot.model.ikcon(robotEndFkine{i});
%                 
%                 robotTraj{i} = jtraj(robotStartFkine{i},robotEndFkine{i},t);
%                 robotJointTrajectory{i} = jtraj(robot.model.getpos(),robotEndIkcon{i},trajectoryLength);
%                 
%             end
%             
%             
%             for trajStep = 1:size(robotJointTrajectory{i},1);
%                 
%                 for i=1:1:numberOfRobots
%                     
%                     robot = robots{i};
%                     jointTrajectory = robotJointTrajectory{i};
%                     
%                     q = jointTrajectory(trajStep,:);
%                     %ur5Q = ur5JointTrajectory(trajStep,:);
%                     
%                     robotFkine = robot.model.fkine(robot.model.getpos());
%                     %ur5Fkine = ur5.model.fkine(ur5.model.getpos());
%                     
%                     if robotBricks{i} ~= 0 % If robot is not currently holding a brick (moving to pick one up)
%                         MoveObject(robotBricks{i},[robotFkine(1,4),robotFkine(2,4),robotFkine(3,4)],0);
%                     end
%                     
%                     robot.model.animate(q);
%                     
%                     drawnow();
%                     
%                 end
%                 
%             end
%             
%         end
    end
end
