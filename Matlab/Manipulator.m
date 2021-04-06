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

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', pi/2, 'offset', -pi/2), ...
                    Revolute('a', -L2, 'd', -W, 'alpha', -pi,'offset', 0), ...
                    Revolute('a', L3, 'd', 0, 'alpha', pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', 0, 'alpha', pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', L4, 'alpha', 0)], 'name', 'Fanuc LR Mate 200iD'); 

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
%% Part A - Calculate the screw axes
w1 = [0; 0; 1];
w2 = [0; 1; 0];
w3 = [0; -1; 0];
w4 = [1; 0; 0];
w5 = [0; 1; 0];
w6 = [0; 0; 1];


p1 = [0; 0; 0];
p2 = [0; 0; L1];
p3 = [0; -W; L2+L1];
p4 = p3;
p5 = p3;
p6 = [L3+L4; -W; L1+L2];


v1 = cross(-w1, p1);
v2 = cross(-w2, p2);
v3 = cross(-w3, p3);
v4 = cross(-w4, p4);
v5 = cross(-w5, p5);
v6 = cross(-w6, p6);

S = [w1 w2 w3 w4 w5 w6;
     v1 v2 v3 v4 v5 v6;];
