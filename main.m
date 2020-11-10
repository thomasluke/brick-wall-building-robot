profile on  % Runs "Profiler" which computational efficiency of all functions and line of code (compute time)

clear all;
clf;

% Creates a log of the command window and clears any previous logs
dfile ='transformsAndStatusLog';
if exist(dfile, 'file') ; delete(dfile); end
diary(dfile)
diary on

% Adding robots to workspace
ur3 = UR3model;
ur5 = LinearUR5(0); 

% Putting robots int a cell container
robots = {ur3,ur5};

tableSurfaceHeight = 0.5;

% Starting positions of the bricks in space
brickPosition = [-0.7, 0.5, tableSurfaceHeight ...
                ;-0.7, -0.5, tableSurfaceHeight ...
                ;-0.4, -0.5, tableSurfaceHeight ...
                ;-0.4, 0.5, tableSurfaceHeight ...
                ;0.9, 0.3, tableSurfaceHeight ...
                ;0.9, 0, tableSurfaceHeight ...
                ;0.9, -0.3, tableSurfaceHeight ...
                ;0.7, 0.3, tableSurfaceHeight ...
                ;0.7, -0.3, tableSurfaceHeight];

% Defines where to position the scene axis and objects
scenePosition = [0,0,tableSurfaceHeight];            

% Creates the scene/environment
disp('Building environment');
bricks = BuildEnvironment(ur3,ur5,brickPosition,scenePosition,0); % ur3 robot, ur5 robot, tableSurfaceHeight, sceneOrientation

% Creating object of the control class
control = control;

% Updating robots into an initial position
disp('Updating robot positions');

numberOfRobotsArray = size(robots);
numberOfRobots = numberOfRobotsArray(end);

for i=1:1:numberOfRobots

robots{i}.model.animate(robots{i}.model.getpos());

end

drawnow();

%% Create plot of the working areas of both the UR3 and UR5
% UR3Pointcloud(ur3);
% UR5Pointcloud(ur5);

%% Building brick wall

brickHeight = 0.06671; % meters

% Defining positions of the bricks for the wall
brickWallPositions = [scenePosition(1)+0.4, scenePosition(2)-0.3, tableSurfaceHeight ...% Add (brickHeight/2) so that end effector move to the centre of the bricks
    ;scenePosition(1)+0.4, scenePosition(2), tableSurfaceHeight ...
    ;scenePosition(1)+0.4, scenePosition(2)+0.3, tableSurfaceHeight ...
    ;scenePosition(1)+0.4, scenePosition(2)-0.3, tableSurfaceHeight+brickHeight ...
    ;scenePosition(1)+0.4, scenePosition(2), tableSurfaceHeight+brickHeight ...
    ;scenePosition(1)+0.4, scenePosition(2)+0.3, tableSurfaceHeight+brickHeight ...
    ;scenePosition(1)+0.4, scenePosition(2)-0.3, tableSurfaceHeight+brickHeight*2 ...
    ;scenePosition(1)+0.4, scenePosition(2), tableSurfaceHeight+brickHeight*2 ...
    ;scenePosition(1)+0.4, scenePosition(2)+0.3, tableSurfaceHeight+brickHeight*2];

% Adding (brickHeight/2) to the z position so that end effector picking positions are at the centre of the bricks

brickPickPositions = brickPosition;

for i = 1:1:size(brickPosition)
    brickPickPositions(i,3)=brickPickPositions(i,3)+(brickHeight/2);
end

brickWallPlacePositions = brickWallPositions;

for i = 1:1:size(brickWallPositions)
    brickWallPlacePositions(i,3)=brickWallPlacePositions(i,3)+(brickHeight/2);
end

%% Control of brick placement sequence. AUtomatically determines which bricks to priortise and whether a brick is out of range (starts with closest bricks 1st and then stops program if bricks at the end are out of range)

BrickPlacementControl(control,robots,bricks,brickPosition,brickPickPositions,brickWallPlacePositions)

%BrickPlacementControl(control,ur3,ur5,bricks,brickPosition,brickPickPositions,brickWallPlacePositions)

profile report
profile off