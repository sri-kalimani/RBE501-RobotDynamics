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
L1 = 0.0465;
L2 = 0.117;
L3 = 0.14959;
L4 = 0.03601;

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha',0), ...
                    Revolute('a', L2, 'd', 0, 'alpha',0), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', L3, 'alpha', pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', L4, 'alpha', 0, 'offset', 0)], 'name', 'Fanuc LR Mate 200iD'); 

% Joint limits
qlim = [-pi  pi;        % q(1)
        -pi/2  pi/2;  % q(2)
        -pi/2  pi/2;  % q(3)              %FIX LIMITS
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/2  pi/2]; % q(6)

% Display the manipulator in the home configuration
q = zeros(1,6);
robot.teach(q);
%% Part A - Calculate the screw axes
% w1 = [0; 0; 1];
% w2 = [0; 1; 0];
% w3 = [0; -1; 0];
% w4 = [1; 0; 0];
% w5 = [0; 1; 0];
% w6 = [0; 0; 1];

w1 = [0; 0; 1];
w2 = [0; 1; 0];
w3 = [0; -1; 0];
w4 = [1; 0; 0];
w5 = [0; -1; 0];
w6 = [1; 0; 0];

p1 = [0; 0; 0];
p2 = [0; 0; L1];
p3 = [0; 0; L2+L1];
p4 = p3;
p5 = [L3; 0; L2+L1];
p6 = p5;
p7 = [L3+L4; 0; L2+L1]; 
% p6 = [L3+L4; -W; L1+L2];

v1 = cross(-w1, p1);
v2 = cross(-w2, p2);
v3 = cross(-w3, p3);
v4 = cross(-w4, p4);
v5 = cross(-w5, p5);
v6 = cross(-w6, p6);

S = [w1 w2 w3 w4 w5 w6;
    v1 v2 v3 v4 v5 v6;];


% S = [ w1 w2 % w3 w4 w5 w6;
%       v1 v2];% v3 v4 v5 v6;];
 %% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration

R = [0 0 1;
     0 1 0;
     1 0 0];
M = [R p7;
     0 0 0 1];

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Test the forward kinematics for 100 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the forward kinematics
        T = fkine(S, M, q)
%       T = robot.fkine(q)
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    %For testing
    T_real = robot.fkine(q)
    T_diff = T - double(T_real)
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');

