% RBE 501 - Robot Dynamics - Spring 2021
% Lab 2
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

%% Part A - Forward Kinematics via PoE in the body frame
% Create the screw axis for each joint in space frame
% Put all the axes into a 6xn matrix S_space, where n is the number of joints

%%YOUR CODE HERE%%

% Let us calculate the homogeneous transformation matrix M for the
% home configuration

%%YOUR CODE HERE%%

% Let us convert the space frame axes into body frame axes

%%YOUR CODE HERE%%

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 10 random sets of joint variables
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
    % Test the correctness of the Forward Kinematics
    %%YOUR CODE HERE%%
    
end
 
fprintf('\nTest passed successfully.\n');


%% Part B - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 10 random sets of joiny
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
    
    % Calculate the Jacobian in the body frame
    %%YOUR CODE HERE%%
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)]; % swap the rotation and translation components
    %%YOUR CODE HERE%%

end

fprintf('\nTest passed successfully.\n');


% Part C - Calculate the Analyical Jacobian of the manipulator
fprintf('---------------------Analytical Jacobian Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Analytical Jacobian for 10 random sets of joint
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
    
    % Calculate the Analytical Jacobian
    %%YOUR CODE HERE%%
    
    if plotOn
        robot.teach(q);
        title('Analytical Jacobian Test');
    end
    
    % Test the correctness of the Analytical Jacobian
    Jref = robot.jacob0(q);
    Jref = Jref(1:3,:);
    %%YOUR CODE HERE%%
    
end

fprintf('\nTest passed successfully.\n');


%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Set the current joint variables
currentQ = zeros(1,3);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S_space,M,currentQ);

% Generate path to follow
t = linspace(0, 2*pi, nTests);
x = 0.25 * cos(t);
y = 0.25 * sin(t);
z = 0.2 * ones(1,nTests);
path = [x; y; z];


if plotOn
    robot.teach(currentQ);
    h = plot_ellipse(J_a*J_a');
    title('Inverse Kinematics Test');
    hold on
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end
     
% Iterate over the target points
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S_body, M, currentQ, 'body');
    currentPose = T(1:3,4);
    
    
    while norm(targetPose - currentPose) > 1e-3
        % YOUR INVERSE KINEMATICS CODE HERE
        % Necessary variables:
        % Current Robot Pose -> currentPose
        % Target Robot Pose ->  targetPose
        % Current Joint Variables -> currentQ
               
        %%YOUR CODE HERE%%
        
        %Update joint variables
        currentQ = currentQ + deltaQ';
        
        %Update current pose
        T = fkine(S_space,M,currentQ,'space');
        %scurrentPose = MatrixLog6(T);
        currentPose = T(1:3,4);
        
        robot.teach(currentQ); 
        plot_ellipse(J_a*J_a',currentPose,'alter',h); 
        drawnow; 

    end
end
fprintf('\nTest passed successfully.\n');