clc; clear; close all;
% Load Peter Corke’s Robotics Toolbox
startup_rvc;

% Define 6-DOF Robot Links
L1 = Link('d', 0.07475 , 'a' , 0.000 , 'alpha', 0);
L2 = Link('d', 0.000 , 'a' , 0.044 , 'alpha', -90*pi/180);
L3 = Link('d', 0.000 , 'a' , 0.1405 , 'alpha', 0);
L4 = Link('d', 0.200 , 'a' , 0.000 , 'alpha', 90*pi/180);
L5 = Link('d', 0.000 , 'a' , 0.000 , 'alpha', -90*pi/180);
L6 = Link('d', 0.028 , 'a' , 0.000 , 'alpha', 90*pi/180);

% Create SerialLink Object
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6-DOF Manipulator');

% Define Start and Goal Pose in Cartesian Space
P1 = [0, 0, 0];  % Start Position (X, Y, Z)
P2 = [0.1, 0.1, 0.1];   % Goal Position (X, Y, Z)

% Convert Cartesian Positions to Transformation Matrices
T1 = transl(P1) * trotx(pi);  % Start Pose with rotation
T2 = transl(P2) * trotx(pi);  % Goal Pose with rotation

% Solve Inverse Kinematics using ikcon() (numerical method)
q0 = zeros(1,6);  % Initial guess
q1 = robot.ikcon(T1, q0);  % Solve IK for (X, Y, Z, Roll, Pitch, Yaw)
q2 = robot.ikcon(T2, q1);  

% Define Time Vector for Trajectory Planning (5 seconds, 100 steps)
t = linspace(0, 5, 100);

% Generate Cubic (3rd-Order) Polynomial Trajectory
[q, qd, qdd] = cubicpolytraj([q1; q2]', [0, 5], t); % Computes cubic trajectory

% Plot Joint Space Trajectories
figure;
subplot(3,1,1); plot(t, q'); xlabel('Time (s)'); ylabel('Joint Angles (rad)'); title('Joint Space Trajectory'); legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6');
subplot(3,1,2); plot(t, qd'); xlabel('Time (s)'); ylabel('Velocity (rad/s)'); title('Joint Velocities');
subplot(3,1,3); plot(t, qdd'); xlabel('Time (s)'); ylabel('Acceleration (rad/s²)'); title('Joint Accelerations');

% Simulate Robot Motion in MATLAB
figure;
robot.plot(q', 'trail', 'k-'); % Show motion with trail
hold on;

% Compute End-Effector Cartesian Trajectory
T_traj = robot.fkine(q');  

% Extract X, Y, Z coordinates properly
x = transl(T_traj); % Extract position (X, Y, Z)

% Plot Cartesian Path of the End Effector
figure;
plot3(x(:,1), x(:,2), x(:,3), 'r-', 'LineWidth', 2);
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('End Effector Trajectory in Cartesian Space');
view(3);
hold on;
% Plot Start and End Points for Verification
scatter3(P1(1), P1(2), P1(3), 100, 'bo', 'filled'); % Start point
scatter3(P2(1), P2(2), P2(3), 100, 'go', 'filled'); % End point
legend('Trajectory', 'Start Position', 'End Position');
hold off;