%% UR3 Model
%  41013 Robotics
%  Thomas Harrison
%  August 2020

% Link('theta',__,'d',__,'a',__,'alpha',__,'offset',__,'qlim',[ ... ])

L1 = Link('d',0.15185,'a',0,'alpha',pi/2,'offset',0)

L2 = Link('d',0,'a',-0.24355,'alpha',0,'offset',0)

L3 = Link('d',0.15,'a',-0.2132,'alpha',0,'offset',0)

L4 = Link('d',0.13105,'a',0,'alpha',pi/2,'offset',0)

L5 = Link('d',0.08535,'a',0,'alpha',-pi/2,'offset',0)

L6 = Link('d',0.0921,'a',0,'alpha',0,'offset',0)

myRobot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UR3')

q = zeros(1,6)

workspace = [-1.5 1.5 -1 1 0 1];                                       % Set the size of the workspace when drawing the robot
scale = 0.4;
myRobot.plot(q,'workspace',workspace,'scale',scale);
