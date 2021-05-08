% RBE 501 - Robot Dynamics - Spring 2021
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: Sriranjani Kalimani
clear, clc, close all
addpath('utils');
plotOn = true;

global S M
%% *** PUT THE FIRST DIGIT OF YOUR WPI ID HERE ***
firstDigitID = 1;


%% Create the manipulator
n = 6; % degrees of freedom

% Robot dimensions (meters)
H1 = 0.320;
H2 = 0.225;
H3 = 0.225;
H4 = 0.065;
W  = 0.035;

if firstDigitID < 5
    robot = SerialLink([Revolute('d', H1, 'a', 0,  'alpha', pi/2, 'offset', 0), ...
        Revolute('d', 0,  'a', H2, 'alpha', 0,     'offset', pi/2), ...
        Revolute('d', -W,  'a', 0,  'alpha', -pi/2,  'offset', -pi/2), ...
        Revolute('d', H3, 'a', 0,  'alpha', pi/2, 'offset', 0), ...
        Revolute('d', 0,  'a', 0,  'alpha', -pi/2,  'offset', 0), ...
        Revolute('d', H4, 'a', 0,  'alpha', 0,     'offset', 0)], ...
        'name', 'Staubli TX-40');
    
    % Joint limits
    qlim = [-180   180;  % q(1)
        -125   125;  % q(2)
        -138   138;  % q(3)
        -270   270;  % q(4)
        -133.5 120;% q(5)
        -270   270]; % q(6)
    
else
    robot = SerialLink([Revolute('d', H1, 'a', 0,  'alpha', -pi/2, 'offset', 0), ...
        Revolute('d', 0,  'a', H2, 'alpha', 0,     'offset', -pi/2), ...
        Revolute('d', W,  'a', 0,  'alpha', pi/2,  'offset', pi/2), ...
        Revolute('d', H3, 'a', 0,  'alpha', -pi/2, 'offset', 0), ...
        Revolute('d', 0,  'a', 0,  'alpha', pi/2,  'offset', 0), ...
        Revolute('d', H4, 'a', 0,  'alpha', 0,     'offset', 0)], ...
        'name', 'Staubli TX-40');
    
    % Joint limits
    qlim = [-180  180;  % q(1)
        -125  125;  % q(2)
        -138  138;  % q(3)
        -270  270;  % q(4)
        -120  133.5;% q(5)
        -270  270]; % q(6)
end

% Display the manipulator in the home configuration
q = zeros(1,n);
robot.teach(q);

currentQ = q;

% Generate and display the circle that the robot has to trace
hold on;

t = linspace(0, 2*pi, 36);
x = 0.3  * ones(1,36);
y = 0.25 * sin(t);
z = 0.25 * cos(t) + 0.4;
path = [x; y; z];

scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Convert cartesian coordinates into twists
targetPose = zeros(6,size(path,2)); % each column of this matrix is a target pose represented by a twist

% Calculate the twist representing the robot's home pose
T = fkine(S, M, currentQ);
currentPose = MatrixLog6(T);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set damping factor
lambda = 0.0000999;

for ii = 1 : size(path,2)
    % First calculate the homogeneous transformation matrix representing
    % each target pose
    T = [eye(3) path(:,ii); 
         0 0 0 1];
     
    % Then perform the matrix logarithm operation to convert transformation
    % matrices into 4x4 elements of se(3)
    t = MatrixLog6(T);
    
    % Finally, "unpack" this matrix (i.e., perform the inverse of the
    % bracket operator)
    targetPose(:,ii) = [t(3,2) t(1,3) t(2,1) t(1:3,4)']';
    
    %% Calculate the inverse kinematics 
    % Starting from the home configuration, iteratively move the robot to each
    % of the poses in the `targetPose` matrix
    while norm(targetPose(:,ii) - currentPose) > 1e-3
        
        % Calculate jacobian
        J = jacob0(S,currentQ); 
       
        % Calculate change in Q
        deltaQ = J' * pinv(J*J' + (lambda^2)*eye(n)) * (targetPose(:,ii) - currentPose);
                
        % Update currentQ
        currentQ = currentQ + deltaQ';     
        
        % Update currentPose
        T = fkine(S, M, currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

        robot.teach(currentQ);
    end
    
end




