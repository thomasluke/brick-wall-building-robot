% startFkine = ur5.model.fkine(ur5.model.getpos())
% 
% startIkcon = ur5.model.ikcon(startFkine)
% 
% desiredPosition = [-0.5,0.2,1] % x, y, z (-y, -z, -x)
% 
% deltaPosition = [(desiredPosition(2)-startFkine(2,4)),  -(desiredPosition(3)-startFkine(3,4)), -(desiredPosition(1)-startFkine(1,4))] % -y, -z, -x
% 
% endFkine = startFkine*transl(deltaPosition);
% 
% endIkcon = ur5.model.ikcon(endFkine)
% 
% 
% t = [0:0.05:2]
% 
% traj = jtraj(startFkine,endFkine,t)
% 
% jointTrajectory = jtraj(ur5.model.getpos(),endIkcon,200);
% 
% for trajStep = 1:size(jointTrajectory,1)
%     q = jointTrajectory(trajStep,:);
%     ur5.model.animate(q);
%     drawnow();
% end
 
 
%ur5.model.animate(endIkcon);

% q0 = [0,0,0,0,0,0,0]
% fkine0 = ur5.model.fkine(q0)
% 
% q1 = [0,-pi/2,0,-pi/2,0,pi/2,0]
% fkine1 = ur5.model.fkine(q1)
% 
% T0 = ur5.model.ikine(fkine0)
% T1 = ur5.model.ikine(fkine1)
% 
% t = [0:0.05:2]
% 
% T2 = jtraj(T0,T1,t)

% for i=1:1:size(T2,1)
% 
%     ur5.model.animate(T2(i,:));
%     drawnow();
% end


% % control = control(ur5,startPoint,endPoint);
% startPoint = [0,0,0];
% endPoint = [0.5,0.5+0.5,0.5];
% 
% % control.MoveEndEffectorToPoint(ur5,startPoint,endPoint)
% 
% startPoint_Ikine = ur5.model.ikine(startPoint);
% endPoint_Ikine = ur5.model.ikine(endPoint);








%% Load the table

hold on

[f,v,data] = plyread('table.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
tableMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');


% %axis equal
% %keyboard
% % clf

%% Load fence model from % https://grabcad.com/library/safety-protections-for-arm-robotic-1
[f,v,data] = plyread('safetyfence.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
safetyfencemesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Get vertex count
fenceVertexCount = size(v,1);

% Move center point to origin
midPoint = sum(v)/fenceVertexCount;
fenceVerts = v - repmat(midPoint,fenceVertexCount,1);

% Create a transform to describe the location (at the origin, since it's centered
fencePose = eye(4);

%  % Then move the object once using Robot Toolbox transforms and without replot
% axis([-10,10,-10,10,-2,2]);

% Move forwards (facing in -y direction)
forwardTR = makehgtform('translate',[0,0,0.5]);

% % Random rotate about Z
 rotateTR = makehgtform('zrotate',0);

% Move the pose forward and a slight and random rotation
fencePose = fencePose * forwardTR*rotateTR;
updatedPoints = [fencePose * [fenceVerts,ones(fenceVertexCount,1)]']';  

% Now update the Vertices
fenceMesh_h.Vertices = updatedPoints(:,1:3);

%% Load the brick

[f,v,data] = plyread('Brick.ply','tri');

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
brickMesh_h = trisurf(f,v(:,1),v(:,2), v(:,3) ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

% Get vertex count
brickVertexCount = size(v,1);

% Move center point to origin
midPoint = sum(v)/brickVertexCount;
brickVerts = v - repmat(midPoint,brickVertexCount,1);

% Create a transform to describe the location (at the origin, since it's centered
brickPose = eye(4);

%  % Then move the object once using Robot Toolbox transforms and without replot
% axis([-10,10,-10,10,-2,2]);

% Move forwards (facing in -y direction)
forwardTR = makehgtform('translate',[0,0,0.5]);

% % Random rotate about Z
 rotateTR = makehgtform('zrotate',pi/2);

% Move the pose forward and a slight and random rotation
brickPose = brickPose * forwardTR*rotateTR;
updatedPoints = [brickPose * [brickVerts,ones(brickVertexCount,1)]']';  

% Now update the Vertices
brickMesh_h.Vertices = updatedPoints(:,1:3);



% 
% %% Flying monkey's head from blender
% % After saving in blender then load the triangle mesh
% [f,v,data] = plyread('monkey.ply','tri');
% 
% % Get vertex count
% monkeyVertexCount = size(v,1);
% 
% % Move center point to origin
% midPoint = sum(v)/monkeyVertexCount;
% monkeyVerts = v - repmat(midPoint,monkeyVertexCount,1);
% 
% % Create a transform to describe the location (at the origin, since it's centered
% monkeyPose = eye(4);
% 
% % Scale the colours to be 0-to-1 (they are originally 0-to-255
% vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
% 
% % Then plot the trisurf
% monkeyMesh_h = trisurf(f,monkeyVerts(:,1),monkeyVerts(:,2), monkeyVerts(:,3) ...
%     ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
% 
% 
% %% To spin the camera 
% if doCameraSpin
%     for ax = -40:5:40; %#ok<UNRCH>
%         for by = [[30:-3:0],[0:3:30]];
%             view(ax,by);
%             drawnow();
%             pause(0.01);
%         end;
%     end
% end
% 
% %% Then move the object once using Robot Toolbox transforms and without replot
% axis([-10,10,-10,10,-2,2]);
% 
% % Move forwards (facing in -y direction)
% forwardTR = makehgtform('translate',[0,-0.01,0]);
% 
% % Random rotate about Z
% randRotateTR = makehgtform('zrotate',(rand-0.5)* 10 * pi/180);
% 
% % Move the pose forward and a slight and random rotation
% monkeyPose = monkeyPose * forwardTR * randRotateTR;
% updatedPoints = [monkeyPose * [monkeyVerts,ones(monkeyVertexCount,1)]']';  
% 
% % Now update the Vertices
% monkeyMesh_h.Vertices = updatedPoints(:,1:3);
% 
% %% Now move it many times
% for i = 1:1000
%     % Random rotate about Z
%     randRotateTR = makehgtform('zrotate',(rand-0.5)* 10 * pi/180);
% 
%     % Move forward then random rotation
%     monkeyPose = monkeyPose * forwardTR * randRotateTR;
% 
%     % Transform the vertices
%     updatedPoints = [monkeyPose * [monkeyVerts,ones(monkeyVertexCount,1)]']';
%     
%     % Update the mesh vertices in the patch handle
%     monkeyMesh_h.Vertices = updatedPoints(:,1:3);
%     drawnow();   
% end
% 
% keyboard
% 
% 




% brickPositionSize = size(brickPosition);
% numberOfBricks = brickPositionSize(1);
% 
% ur3MaxReachDistance = 0.5; % Value defined in the UR data sheet
% ur5MaxReachDistance = 0.85; % Value defined in the UR data sheet
% 
% reachDistances = zeros(numberOfBricks,3);
% 
% ur3BasePosition = (ur3.model.base(1:3,4))'; % ' = transpose so that the midPoint and base point coordinates are oriented the same way
% ur5BasePosition = (ur5.model.base(1:3,4))';
% 
% %ur3BasePosition(3) = ur3BasePosition(3)+0.1519;
% 
% for i=1:1:numberOfBricks
%     v = bricks{i}.Vertices;
%     objectVertexCount = size(v,1);
%     midPoint = sum(v)/objectVertexCount;
%     ur3ReachDistance = norm(midPoint-ur3BasePosition);
%     
%     ur5ReachDistanceArray=zeros(1,10);
%     ur5BasePosition = (ur5.model.base(1:3,4))';
%     
%     for j=0:1:9  % Allowing for min distances accounting for sthe rail movement along the x axis
%         ur5BasePosition(1) = ur5BasePosition(1)-(0.1);
%         
%         ur5ReachDistanceArray(j+1) = norm(midPoint-ur5BasePosition);
%     end
%     
%     ur5ReachDistance = min(ur5ReachDistanceArray);
%     reachDistances(i,1:3)=[i,ur3ReachDistance,ur5ReachDistance];
% end
% 
% 
% while size(reachDistances)>0
%     [ur3ClosestBrickDistance,i1] = min(reachDistances(:,2));
%     
%     
%     if ur3ClosestBrickDistance < ur3MaxReachDistance
%         
%         disp("Brick within reach for UR3")
%         
%         
%         i = reachDistances(i1,1);
%         
%         reachDistances(i1,:) = [];
%         
%         ur3PickPosition = brickPickPositions(i,1:3);
%         ur3PlacePosition = brickWallPlacePositions(1,1:3)
%         brickWallPlacePositions(1,:) = [];
%         
%         ur3Brick = bricks{i};
%         
%     else
%         
%         disp("No bricks within reach for UR3")
%         
%         ur3PickPosition = ur3.model.fkine([0,-pi/2,0,0,0,0]);
%         ur3PlacePosition = ur3.model.fkine([0,-pi/2,0,0,0,0]);
%         
%         ur3Brick = 0;
%         
%     end
%     
%     [ur5ClosestBrickDistance,i2] = min(reachDistances(:,3));
%     
%     if ur5ClosestBrickDistance < ur5MaxReachDistance & size(reachDistances) > 1
%         
%         disp("Brick within reach for UR5")
%         
%         
%         i = reachDistances(i2,1);
%         
%         reachDistances(i2,:) = [];
%         
%         ur5PickPosition = brickPickPositions(i,1:3);
%         ur5PlacePosition = brickWallPlacePositions(1,1:3);
%         brickWallPlacePositions(1,:) = [];
%         
%         ur5Brick = bricks{i};
%         
%     else
%         
%         disp("No bricks within reach for UR5")
%         
%         %q = [0,0,0,0,0,0,0]
%         ur5PickPosition = ur5.model.fkine([0,0,0,0,0,0,0]);
%         ur5PlacePosition = ur5.model.fkine([0,0,0,0,0,0,0]);
%         
%         ur5Brick = 0;
%         
%     end
%     
%     if ur3ClosestBrickDistance > ur3MaxReachDistance & ur5ClosestBrickDistance > ur5MaxReachDistance
%         disp("Remaining bricks out of reach for all robots")
%         break;
%     end
%     
%     disp("Picking up bricks")
%     control.MoveEndEffectorToPoint(ur3,ur5,0,0,ur3PickPosition,ur5PickPosition);
%     
%     disp("Placing bricks")
%     control.MoveEndEffectorToPoint(ur3,ur5,ur3Brick,ur5Brick,ur3PlacePosition,ur5PlacePosition);
%     
% end
% 
% disp("Brick wall assembly complete")


% for i=1:2:numberOfBricks
%     if i<numberOfBricks
%         disp("Picking up bricks " + i + " and " + (i+1))
%         control.MoveEndEffectorToPoint(ur3,ur5,0,0,brickPickPositions(i,1:3),brickPickPositions(i+1,1:3));
%
%         disp("Placing bricks " + i + " and " + (i+1))
%         control.MoveEndEffectorToPoint(ur3,ur5,bricks{i},bricks{i+1},brickWallPlacePositions(i,1:3),brickWallPlacePositions(i+1,1:3));
%
%                   %  ur3Fkine = ur3.model.fkine(ur3.model.getpos());
%
%         %LoadObject("Brick.ply",[ur3Fkine(1,4),ur3Fkine(2,4),ur3Fkine(3,4)],pi/2)
%
% %         MoveObject(bricks{i},[ur3Fkine(1,4),ur3Fkine(2,4),ur3Fkine(3,4)],0)
%     else
%         disp("Picking up brick " + i)
%         control.MoveEndEffectorToPoint(ur3,ur5,0,0,brickPickPositions(i,1:3),[-0.5,0,1.3]);
%
%         disp("Placing brick " + i)
%         control.MoveEndEffectorToPoint(ur3,ur5,bricks{i},bricks{i+1},brickWallPlacePositions(i,1:3),[-0.5,0,1.3]);
%
%     end
% end

% Control of the robot arms

% for i=1:1:size(brickPosition)
%     if i<9
%         disp("Picking up brick " + i)
%         control.MoveEndEffectorToPoint(ur3,ur5,brickPosition(i,1:3),[-0.5,0,1.3]);
%
%         disp("Placing brick " + i)
%         control.MoveEndEffectorToPoint(ur3,ur5,brickWallPositions(i,1:3),[-0.5,0,1.3]);
%     else
%         disp("Picking up brick " + i)
%         control.MoveEndEffectorToPoint(ur3,ur5,brickPosition(i,1:3),[-0.5,0,1.3]);
%
%         disp("Placing brick " + i)
%         control.MoveEndEffectorToPoint(ur3,ur5,brickWallPositions(i,1:3),[-0.5,0,1.3]);
%     end
% end


% for i=1:2:size(brickPosition)
%     if i<9
%         control.MoveEndEffectorToPoint(ur3,ur5,brickPosition(i,1:3),brickPosition(i+1,1:3));
%         control.MoveEndEffectorToPoint(ur3,ur5,brickWallPositions(i,1:3),brickWallPositions(i+1,1:3));
%     else
%         control.MoveEndEffectorToPoint(ur3,ur5,brickPosition(i,1:3),[-0.5,0,1.3]);
%         control.MoveEndEffectorToPoint(ur3,ur5,brickWallPositions(i,1:3),[-0.5,0,1.3]);
%     end
% end





% ur3EndPoint = [0.75,-0.3,0.7];
%
% ur5EndPoint = [-0.75,-0.3,0.7];
%
% control.MoveEndEffectorToPoint(ur3,ur5,ur3EndPoint,ur5EndPoint)

% endPoint = [0.2,-0.2,0.7];
% control.MoveEndEffectorToPoint(ur5,endPoint)
%
% endPoint = [0.-0.3,0,0.5];
% control.MoveEndEffectorToPoint(ur5,endPoint)


%workspace = [-2.5 2.5 -1.5 1.5 0 4];                                       % Set the size of the workspace when drawing the robot
%workspace = [-1.5 1.5 -1 1 0 1.5];                                       % Set the size of the workspace when drawing the robot

%axis([-2.5 2.5 -1.5 1.5 0 4])

%hold on

%
% %ur3_base = transl(0.75,0,0.5); % x, y, z
% drawnow();
% ur3.model.base = ur3.model.base * transl(0.75,0,0.5);
%
% %ur5_base = transl(-0.75,0,0.5); % x, y, z
% drawnow();
% ur5.model.base = ur5.model.base* transl(0,0.5,0); % y, z, x
%
% q = [pi/2,-pi/2,0,-pi/2,0,0]
% %ur3.model.animate(q)
% % ur5.model.animate([0,0,0,0,0,0,0])
% drawnow();
%
% for i=0:-pi/180:-pi/2
%
%     %ur3.model.animate([pi/2+i,-pi/2-i,i,-pi/2-i,i,i]);
%   % ur5.model.animate([0,-pi/2-i,i,-pi/2-i,i,i,i]);
%
%     drawnow();
% end