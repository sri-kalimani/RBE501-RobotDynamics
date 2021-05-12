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
        -3*pi/4  pi/4;  % q(3)              %FIX LIMITS
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/2  pi/2]; % q(6)

% Display the manipulator in the home configuration
q = zeros(1,6);
robot.teach(q);
%% Part A - Calculate the screw axes
w1 = [0; 0; 1];
w2 = [0; 1; 0];
w3 = [0; 1; 0];
w4 = [1; 0; 0];
w5 = [0; 1; 0];
w6 = [1; 0; 0];

p1 = [0; 0; 0];
p2 = [0; 0; L1];
p3 = [0; 0; L2+L1];
p4 = [L3; 0; L2+L1];
p5 = [L3; 0; L2+L1];
p6 = [L3; 0; L2+L1];
pe = [L3+L4; 0; L2+L1]; 

v1 = cross(-w1, p1);
v2 = cross(-w2, p2);
v3 = cross(-w3, p3);
v4 = cross(-w4, p4);
v5 = cross(-w5, p5);
v6 = cross(-w6, p6);

S = [w1 w2 w3 w4 w5 w6;
     v1 v2 v3 v4 v5 v6;];

R = [0 0 1;
     0 -1 0;
     1 0 0];
M = [R pe;
     0 0 0 1];

[m, n] = size(S);
S_body = [];

for i = 1:6
    S_body(:, i) = twistspace2body(S(1:6, i), M);
end


%% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration

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
        T = fkine(S, M, q);
       %T = robot.fkine(q);
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    %For testing
    T_real = robot.fkine(q);
    T_diff = T - double(T_real);
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
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
             qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
             qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
             qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
             qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
     
    
    % Calculate the Jacobian in the body frame    
    J_b = jacobe(S,M,q);
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)]; % swap the rotation and translation components
    assert(all(all(abs(double(robot.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

 
%% Part C - Calculate the Analyical Jacobian of the manipulator
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
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the Analytical Jacobian
    J_a = jacoba(S,M,q); % S in space
    
    if plotOn
        robot.teach(q);
        title('Analytical Jacobian Test');
    end
    
    % Test the correctness of the Jacobian
    Jref = robot.jacob0(q);
    Jref = Jref(1:3,:);
    assert(all(all(abs(double(Jref) - J_a) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Set the current joint variables
currentQ = zeros(1,6);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S,M,currentQ);

% Generate path to follow
t = linspace(0, pi, nTests);
x = 0.25 * sin(t);
y = 0.25 * cos(t);
z = 0.1 * ones(1,nTests);
path = [x; y; z];
lam =  0.1;          %tryna do ikin with this

if plotOn
    robot.teach(currentQ);
    %h = plot_ellipse(J_a*J_a');
    %title('Inverse Kinematics Test');
    %hold on
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end
     
% Iterate over the target points
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S, M, currentQ);
    currentPose = T(1:3,4);
    
    
    while norm(targetPose - currentPose) > 1e-3
        % YOUR INVERSE KINEMATICS CODE HERE
        % Necessary variables:
        % Current Robot Pose -> currentPose
        % Target Robot Pose ->  targetPose
        % Current Joint Variables -> currentQ
               
        J_a = jacoba(S,M,currentQ);

        %Calculate delatQ using damped least squares 
        psedo = J_a*(J_a)' + lam^2*eye(3);
        J_a = J_a'*pinv(psedo) ;
        deltaQ = J_a*(targetPose - currentPose);
        
        %Update joint variables
        currentQ = currentQ + deltaQ';
        
        %Update current pose
        T = fkine(S,M,currentQ);
%         scurrentPose = MatrixLog6(T);
        currentPose = T(1:3,4);
        
        robot.teach(currentQ);
        %clf
        %h = plot_ellipse(J_a*J_a', currentPose);
    end
end
fprintf('\nTest passed successfully.\n');


