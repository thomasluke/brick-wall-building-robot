clear all
clf

% Adding robots to workspace
robot = UR3model;

% Loading in rosbag data
bag = rosbag('RosBag.bag');
