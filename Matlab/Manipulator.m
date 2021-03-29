% RBE 501 - Robot Dynamics - Spring 2021
% Project
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
clear, clc, close all
addpath('utils');

plotOn = true;
nTests = 20; % number of random test configurations

%% Create the manipulator
% robot length values (meters)
L0 = 0.3;
L1 = 0.3;
L2 = 0.3;
L3 = 0.3;
L4 = 0.3;
W = 0.1;

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', -pi/2, 'offset', pi/2), ...
                    Revolute('a', 0, 'd', L2, 'alpha', 0), ...
                    Revolute('a', 0, 'd', W, 'alpha', pi/2), ...
                    Revolute('a', L3, 'd', 0, 'alpha', -pi/2), ...
                    Revolute('a', L4, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', 0)], 'name', 'Fanuc LR Mate 200iD'); 

% Joint limits
qlim = [-pi/2  pi/2;  % q(1)
        -pi/4  pi/2;  % q(2)
        0      pi/3;  % q(3)
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/2  pi/2]; % q(6)

% Display the manipulator in the home configuration
q = zeros(1,6);
robot.teach(q);