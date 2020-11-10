function BrickPlacementControl(control,robots,bricks,brickPosition,brickPickPositions,brickWallPlacePositions)
% BrickPlacementControl controls brick placement sequence. Automatically 
% determines which bricks to priortise and whether a brick is out of range 
% (starts with closest bricks 1st and then stops program if bricks at the end are out of range)

ur3 = robots{1};
ur5 = robots{2};

brickPositionSize = size(brickPosition);
numberOfBricks = brickPositionSize(1);

% Defining maximum reach distances of the robots
ur3MaxReachDistance = 0.5; % Value defined in the UR data sheet
ur5MaxReachDistance = 0.85; % Value defined in the UR data sheet

reachDistances = zeros(numberOfBricks,3);

ur3BasePosition = (ur3.model.base(1:3,4))'; % ' = transpose so that the midPoint and base point coordinates are oriented the same way
ur5BasePosition = (ur5.model.base(1:3,4))';

%ur3BasePosition(3) = ur3BasePosition(3)+0.1519;

% Create "reachDistances" matrix which contains the minimum ditances from
% both the UR3 and UR5 end effectors to each brick. This is used to
% determine which robot should target each brick and whether the bricks are
% within reach range.
for i=1:1:numberOfBricks
    v = bricks{i}.Vertices;
    objectVertexCount = size(v,1);
    midPoint = sum(v)/objectVertexCount;
    ur3ReachDistance = norm(midPoint-ur3BasePosition);
    
    ur5ReachDistanceArray=zeros(1,10);
    ur5BasePosition = (ur5.model.base(1:3,4))';
    
    for j=0:1:9  % Allowing for min distances accounting for sthe rail movement along the x axis
        ur5BasePosition(1) = ur5BasePosition(1)-(0.1);
        
        ur5ReachDistanceArray(j+1) = norm(midPoint-ur5BasePosition);
    end
    
    ur5ReachDistance = min(ur5ReachDistanceArray);
    
    % Contains the minimum ditances from both the UR3 and UR5 end effectors to each brick
    reachDistances(i,1:3)=[i,ur3ReachDistance,ur5ReachDistance];
end

% While not all bricks have been picked and placed
while size(reachDistances)>0
    
    % Target closest brick. Store distance and brick number
    [ur3ClosestBrickDistance,i1] = min(reachDistances(:,2));
    
    % Check that the closest brick to the UR3 is within its max reach range
    if ur3ClosestBrickDistance < ur3MaxReachDistance
        
        disp("UR3 PROCEED: Brick within reach for UR3")
        
        
        i = reachDistances(i1,1);
        
        % Remove brick from the reachDistances array as it has now been picked
        reachDistances(i1,:) = [];
        
        
        ur3PickPosition = brickPickPositions(i,1:3);
        
        % Place brick in the next brick wall position
        ur3PlacePosition = brickWallPlacePositions(1,1:3);
        
        % Remove complete brick wall position
        brickWallPlacePositions(1,:) = [];
        
        ur3Brick = bricks{i};
    
    % If closest brick to the UR3 is not within its max reach range then do not move to a wait at the position set below
    else
        
        disp("UR3 STNDABY: No bricks within reach for UR3")
        
        ur3PickPosition = ur3.model.fkine([0,0,0,0,0,0]);
        ur3PlacePosition = ur3.model.fkine([0,0,0,0,0,0]);
        
        ur3PickPosition = ur3PickPosition(1:3,4);
        ur3PlacePosition = ur3PlacePosition(1:3,4);
        
        ur3Brick = 0;
        
    end
    
    [ur5ClosestBrickDistance,i2] = min(reachDistances(:,3));
    
    a = size(reachDistances);
    sizeReachDistances = a(end);
    
    if ur5ClosestBrickDistance < ur5MaxReachDistance & sizeReachDistances > 0
        
        disp("UR5 PROCEED: Brick within reach for UR5")
        
        
        i = reachDistances(i2,1);
        
        reachDistances(i2,:) = [];
        
        ur5PickPosition = brickPickPositions(i,1:3);
        ur5PlacePosition = brickWallPlacePositions(1,1:3);
        brickWallPlacePositions(1,:) = [];
        
        ur5Brick = bricks{i};
        
    else
        
        disp("UR5 STANDBY: No bricks within reach for UR5")
        
        %q = [0,0,0,0,0,0,0]
        ur5PickPosition = ur5.model.fkine([0,0,0,0,0,0,0]);
        ur5PlacePosition = ur5.model.fkine([0,0,0,0,0,0,0]);
        
        ur5PickPosition = ur5PickPosition(1:3,4);
        ur5PlacePosition = ur5PlacePosition(1:3,4);
        
        ur5Brick = 0;
        
    end
    
    % If no bricks are within max reach range range for either the UR3 or UR5 then display message
    if ur3ClosestBrickDistance > ur3MaxReachDistance & ur5ClosestBrickDistance > ur5MaxReachDistance
        disp("ALL ROBOTS STANDBY: Remaining bricks out of reach for all robots")
        break;
    end
    
    %disp(['Picking up bricks at [',num2str(ur3PickPosition(1)),', ',num2str(ur3PickPosition(2)),', ',num2str(ur3PickPosition(3)),'] and [',num2str(ur3PickPosition(1)),', ',num2str(ur3PickPosition(2)),', ',num2str(ur3PickPosition(3)),']']);
    disp("Picking up bricks")
    % Picking bricks (not currently holding bricks). 0,0 means that no bricks will move along the end effector
    control.MoveEndEffectorToPoint(robots,0,0,ur3PickPosition,ur5PickPosition);
    
    disp("Placing bricks")
    % Placing bricks (holding bricks). ur3Brick,ur5Brick means that bricks will move along the end effector
    control.MoveEndEffectorToPoint(robots,ur3Brick,ur5Brick,ur3PlacePosition,ur5PlacePosition);
    
%     robotBricks = {ur3Brick,ur5Brick};
%     robotPickPositions = {ur3PickPosition;ur5PickPosition};
%     
%     disp("Picking up bricks")
%     control.MoveEndEffectorToPoint(robots,{0,0},robotPickPositions);
%     
%     disp("Placing bricks")
%     control.MoveEndEffectorToPoint(robots,robotBricks,robotPickPositions);
    
end

disp("Brick wall assembly complete")

end

