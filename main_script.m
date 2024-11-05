%% Main Script
% Darius Irani
clc; clear all;

%% Defining Variables
% User specifies number of robots
numRobots = input('Enter the number of robots: ');

% gets user specified boundary coordinates from function
[boundaryX, boundaryY, obstacleCenters, obstacleRadii] = get_boundary();


