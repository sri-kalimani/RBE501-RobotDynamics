% RBE 501 - Robot Dynamics - Spring 2021
% Lab 3
% Worcester Polytechnic Institute
%
% Original Creator: L. Fichera <lfichera@wpi.edu>
% Modified By: Kevin Boenisch, Meha Mohapatra, Nicole Kuberka and Sriranjani Kalimani
clear, clc, close all
addpath('utils');

plotOn = false;

% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Create the manipulator
n = 3; % degrees of freedom

L1 = 0.095; % Lenght of Link 1 [m]
L2 = 0.120; % Lenght of Link 2 [m]
L34 = 0.1856; % Lenght of Link 3 [m]

m1 = 0.0991;   % Mass of Link 1 [kg]
m2 = 0.0875;   % Mass of Link 2 [kg]
m3 = 0.0778;   % Mass of Link 3 [kg]

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', -pi/2, 'offset', 0), ...
                    Revolute('a', L2, 'd', 0, 'alpha', 0, 'offset', -pi/2), ...
                    Revolute('a', L34, 'd', 0, 'alpha', pi/2, 'offset', pi/2)], ...
                    'name', 'RBE501 Lab Robot');

% Display the manipulator in the home configuration
q = zeros(1,3);
robot.plot(q);

%% Robot Definition
% Let us create our screw axes - note that only 3 joints matter
%%YOUR CODE HERE%%

% Let us create our home configuration matrix
%%YOUR CODE HERE%%

%Let us create transforms between links

%%YOUR CODE HERE%%
M01 = ...; %Pose of link {1} in frame {0}
M12 = ...; %Pose of link {2} in frame {1}
M23 = ...; %Pose of link {3} in frame {2}
M34 = ...; %Pose of link {4} in frame {3}

M1 = M01;       %Pose of link {1} in frame {0}
M2 = M1 * M12;  %Pose of link {2} in frame {0}
M3 = M2 * M23;  %Pose of link {3} in frame {0}
M4 = M3 * M34;  %Pose of the body frame {4} in frame {0}

% Create a single matrix - will be useful later
Mlist = cat(3, M01, M12, M23, M34);

% Spatial Inertia Matrices
Ib1 = 1e-9 * [77132.67 -181.32 -5500.03;
                -181.32 63015.08 1155.98;
                -5500.03 1155.98 70263.49];
Ib2 = 1e-9 * [122510.29 -0.01 0.04;
                -0.01 115999.61 1817.51;
                0.04 1817.51 17810.74];
Ib3 = 1e-9 * [15196.78 -469.99 -10490.12;
                 -469.99 206802.72 22.98;
                 -10490.12 22.98 209462.11];
G1 = ...; %Spatial Inertia Matrix for link 1
G2 = ...; %Spatial Inertia Matrix for link 2
G3 = ...; %Spatial Inertia Matrix for link 3

% Create a single matrix - will be useful later
Glist = cat(3, G1, G2, G3);


%% I - Forward Dynamics -- robot falls under gravity
fprintf('---------------Simulation of Robot Falling Under Gravity----------------\n');

q0 = zeros(3,1);      % initial joint variables
qd0 = zeros(3,1);     % initial joint velocities
taumat = zeros(40,3); % joint torques (no torque being applied)
Ftipmat = zeros(size(taumat, 1), 6); % end effector wrench (no force/moment being applied)
dt = 0.05;            % time step
intRes = 8;           % Euler integration step

% We will now use the `ForwardDynamicsTrajectory` to simulate the robot
% motion.
[qt,~] = ForwardDynamicsTrajectory(q0, qd0, taumat, g, ...
                                   Ftipmat, Mlist, Glist, S, dt, ...
                                   intRes);

title('Robot Falling Under Gravity');                               
robot.plot(qt);

input('Simulation complete. Press Enter to continue.');


%% II - Inverse Dynamics: Gravity Compensation
% We will now calculate the joint torques necessary for the robot to stay
% in place, i.e., not fall under its own weight.
fprintf('---------------------------Gravity Compensation-------------------------\n');

q0 = zeros(3,1);  % initial configuration 
qd0 = zeros(3,1); % initial velocities

% Invoke the `GravityForces` function. This function calculates the inverse
% dynamics of the robot when the robot is not moving and the only force
% acting on it is gravity.
grav = GravityForces(q0, g, Mlist, Glist, S);

fprintf('Joint Torques: ');
fprintf('[%f %f %f] Nm\n', grav(1), grav(2), grav(3));

% Simulate for 5 seconds
tf = 5;           % total simulation time
dt = 0.05;        % time step
taumat = repmat(grav', [tf/dt 1]);   % apply only the torque required for gravity comp
Ftipmat = zeros(size(taumat, 1), 6); % end effector wrench
intRes = 8;       % Euler integration step

% Invoke the `ForwardDynamicsTrajectory` to simulate the robot motion.
[qt,~] = ForwardDynamicsTrajectory(q0, qd0, taumat, g, ...
                                   Ftipmat, Mlist, Glist, S, dt, ...
                                   intRes);

title('Gravity Compensation');
robot.plot(qt);

input('Simulation complete. Press Enter to continue.');


%% Dynamics III - Control the motion between set points
fprintf('----------------------Dynamic Control of a 3-DoF Arm--------------------\n');

% We will first generate a path in task space and solve the inverse
% kinematics for each of these points. 

% Initialize the matrix to store the IK result
nPts = 10;
targetQ = zeros(3,nPts);

% Set the current joint variables
currentQ = zeros(1,3);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S,M,currentQ);

% Generate a path to follow
fprintf('Generating task space path... ');
t = linspace(0, 2*pi, nPts);
x = 0.25 * cos(t);
y = 0.25 * sin(t);
z = 0.2 * ones(1,nPts);
path = [x; y; z];
fprintf('Done.\n');

fprintf('Calculating the Inverse Kinematics... ');
robot.plot(currentQ);
hold on
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Iterate over the target points
for ii = 1 : nPts
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S,M,currentQ);
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        J_a = jacoba(S,M,currentQ);
        
        % Use the Levenberg-Marquadt algorithm (Damped Least Squares)
        lambda = 0.5;
        deltaQ = J_a' * pinv(J_a*J_a' + lambda^2 * eye(3)) * (targetPose - currentPose);
                    
        currentQ = currentQ + deltaQ';       
        T = fkine(S,M,currentQ);
        currentPose = T(1:3,4);
    end
    
    targetQ(:,ii) = currentQ;
end

fprintf('Done.\n');


% Now, for each pair of consecutive set points, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generate the Joint Torque Profiles... ');

% Initialize the arrays where we will accumulate the output of the robot
% dynamics, so that we can display it later
qtt = []; % Joint Variables
tau = [];

for jj = 1 : nPts - 1
    t0 = 0; tf = 0.5; % Starting and ending time of each trajectory
    N = 500;          % Number of intermediate setpoints
    t = linspace(t0, tf, N); % time vector
    
    q = zeros(n,N);   % joint variables
    qd = zeros(n,N);  % joint velocities
    qdd = zeros(n,N); % joint accelerations
    
    for ii = 1 : n
        % Calculate the coefficients of the quintic polynomial
        a = quinticpoly(t0, tf, ...
            targetQ(ii,jj), targetQ(ii,jj+1), ...
            0, 0, 0, 0);
        
        % Generate the joint profiles (position, velocity, and
        % acceleration)
        q(ii,:) = a(1) + a(2) * t + a(3) * t.^2 + a(4) * t.^3 + a(5) * t.^4 + a(6) * t.^5;
        qd(ii,:) = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
        qdd(ii,:) = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
    end
    
    % Use the equations of motion to calculate the necessary torques to trace
    % the trajectory
    Ftipmat = zeros(N,6); % no end effector force
    taumat = InverseDynamicsTrajectory(q', qd', qdd', ...
        g, Ftipmat, Mlist, Glist, S);
    
    % Use the Forward Dynamics to simulate the robot behavior
    dt = tf/N;  % time step
    intRes = 1; % Euler integration constant
    [qt, qdt] = ForwardDynamicsTrajectory(q(:,1), qd(:,1), taumat, g, ...
        Ftipmat, Mlist, Glist, S, dt, ...
        intRes);
    
    qtt = [qtt; qt]; % Accumulate the results
    tau = [tau; taumat];
end

fprintf('Done.\n');

fprintf('Simulate the robot...');
title('Inverse Dynamics Control');
robot.plot(qtt(1:10:end,:));
fprintf('Done.\n');

% Display the Joint Torques
figure, hold on, grid on
plot((1:length(tau))*dt, tau(:,1), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,2), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,3), 'Linewidth', 2);
xlim([0 max((1:length(tau))*dt)]);
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3'});
set(gca, 'FontSize', 14);

fprintf('Program completed successfully.\n');


%% Part g) - Control the motion between set points with 1kg payload
fprintf('----------------------Dynamic Control of a 3-DoF Arm v2--------------------\n');
  
% Create a single matrix - will be useful later
Glist = cat(3, G1, G2, G3);

% We will first generate a path in task space and solve the inverse
% kinematics for each of these points. 

% Initialize the matrix to store the IK result
nPts = 10;
targetQ = zeros(3,nPts);

% Set the current joint variables
currentQ = zeros(1,3);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S,M,currentQ);

% Generate a path to follow
fprintf('Generating task space path... ');
t = linspace(0, 2*pi, nPts);
x = 0.25 * cos(t);
y = 0.25 * sin(t);
z = 0.2 * ones(1,nPts);
path = [x; y; z];
fprintf('Done.\n');

fprintf('Calculating the Inverse Kinematics... ');
robot.plot(currentQ);
hold on
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Iterate over the target points
for ii = 1 : nPts
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S,M,currentQ);
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-3
        J_a = jacoba(S,M,currentQ);
        
        % Use the Levenberg-Marquadt algorithm (Damped Least Squares)
        lambda = 0.5;
        deltaQ = J_a' * pinv(J_a*J_a' + lambda^2 * eye(3)) * (targetPose - currentPose);
                    
        currentQ = currentQ + deltaQ';       
        T = fkine(S,M,currentQ);
        currentPose = T(1:3,4);
    end
    
    targetQ(:,ii) = currentQ;
end

fprintf('Done.\n');


% Now, for each pair of consecutive set points, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generate the Joint Torque Profiles... ');

% Initialize the arrays where we will accumulate the output of the robot
% dynamics, so that we can display it later
qtt = []; % Joint Variables
tau = [];

for jj = 1 : nPts - 1
    t0 = 0; tf = 0.5; % Starting and ending time of each trajectory
    N = 500;          % Number of intermediate setpoints
    t = linspace(t0, tf, N); % time vector
    
    q = zeros(n,N);   % joint variables
    qd = zeros(n,N);  % joint velocities
    qdd = zeros(n,N); % joint accelerations
    
    for ii = 1 : n
        % Calculate the coefficients of the quintic polynomial
        a = quinticpoly(t0, tf, ...
            targetQ(ii,jj), targetQ(ii,jj+1), ...
            0, 0, 0, 0);
        
        % Generate the joint profiles (position, velocity, and
        % acceleration)
        q(ii,:) = a(1) + a(2) * t + a(3) * t.^2 + a(4) * t.^3 + a(5) * t.^4 + a(6) * t.^5;
        qd(ii,:) = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
        qdd(ii,:) = 2*a(3) + 6*a(4)*t + 12*a(5)*t.^2 + 20*a(6)*t.^3;
    end
    
    % Use the equations of motion to calculate the necessary torques to trace
    % the trajectory
    Ftipmat = zeros(N, 6);
    %Iterate through each point to build up wrench matrix
    for ii = 1 : N
        %%YOUR CODE HERE%%
    end
        
    taumat = InverseDynamicsTrajectory(q', qd', qdd', ...
        g, Ftipmat, Mlist, Glist, S);
    
    % Use the Forward Dynamics to simulate the robot behavior
    dt = tf/N;  % time step
    intRes = 1; % Euler integration constant
    [qt, qdt] = ForwardDynamicsTrajectory(q(:,1), qd(:,1), taumat, g, ...
        Ftipmat, Mlist, Glist, S, dt, ...
        intRes);
    
    qtt = [qtt; qt]; % Accumulate the results
    tau = [tau; taumat];
end

fprintf('Done.\n');

fprintf('Simulate the robot...');
title('Inverse Dynamics Control');
robot.plot(qtt(1:10:end,:));
fprintf('Done.\n');

% Display the Joint Torques
figure, hold on, grid on
plot((1:length(tau))*dt, tau(:,1), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,2), 'Linewidth', 2); 
plot((1:length(tau))*dt, tau(:,3), 'Linewidth', 2);
xlim([0 max((1:length(tau))*dt)]);
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3'});
set(gca, 'FontSize', 14);

fprintf('Program completed successfully.\n');