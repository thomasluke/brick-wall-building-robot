clear all
clf

robot = UR3model;

q=[pi/6,0,-pi/2,0,0,0]

workspace = [-0.3 0.3 -0.3 0.3 -1 1];                                       % Set the size of the workspace when drawing the robot
scale = 0.5;

robot.model.teach;