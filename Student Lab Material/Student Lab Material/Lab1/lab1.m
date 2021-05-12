% RBE 501 - Robot Dynamics - Spring 2021
% Lab 1
% Worcester Polytechnic Institute
%
% Original Creator: L. Fichera <lfichera@wpi.edu>
% Modified By: Kevin Boenisch, Meha Mohapatra, Nicole Kuberka and Sriranjani Kalimani
clear, clc, close all
addpath('utils');

plotOn = true;
nTests = 20; % number of random test configurations

%% Create the manipulator
% robot length values (meters)
L1 = 0.095;
L2 = 0.120;
L3 = 0.150;
L4 = 0.0356;

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha',-pi/2), ...
                    Revolute('a', L2, 'd', 0, 'alpha',0, 'offset', -pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', L3, 'alpha', pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', 0), ...
                    Revolute('a', 0, 'd', L4, 'alpha', 0, 'offset', 0)], 'name', 'RBE501 Lab Robot'); 

% Joint limits
qlim = [-pi/2  pi/2;      % q(1)
        -pi/2  pi/2;  % q(2)
        -3*pi/4  pi/4;  % q(3)             
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/2  pi/2]; % q(6)

% Display the manipulator in the home configuration
q = zeros(1,6);
robot.teach(q);

%% Part A - Calculate the screw axes and home configuration
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints


%% Part B - Calculate the forward kinematics with the Product of Exponentials formula
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
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the forward kinematics
    %%YOUR CODE HERE%%
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    %Explain what this is doing and how it works
    T_real = robot.fkine(q);
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
fprintf('\nTest passed successfully.\n');


%% Part C - Calculate the Space Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 100 random sets of joint
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the Forward Kinematics
    %%YOUR CODE HERE%%    
    
    % Calculate the Jacobian
    %%YOUR CODE HERE%%
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
    assert(all(all(abs(double(robot.jacob0(q)) - Jcoords) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,3);

if plotOn
    robot.teach(currentQ);
    h = triad('matrix', M, 'tag', 'Target Pose', 'linewidth', 2.5, 'scale', 0.5);
end
     
% Generate the test configurations
q = [linspace(0,pi/2,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);];

for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate the robot's pose
    T = fkine(S,M,q(:,ii)');
    targetPose = MatrixLog6(T);
    targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';
    
    if plotOn
        set(h, 'matrix', T);
        title('Inverse Kinematics Test');
        drawnow;
    end
    
    % Inverse Kinematics
    while norm(targetPose - currentPose) > 1e-3
        
        %%YOUR CODE HERE%%
        
        currentQ = currentQ + deltaQ';

        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
        
        if plotOn
            try
                robot.teach(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end
end

fprintf('\nTest passed successfully.\n');