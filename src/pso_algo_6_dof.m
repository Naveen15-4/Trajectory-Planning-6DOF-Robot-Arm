clc; clear; close all;

% Set random seed for varying outputs
rng('shuffle');

% Load Peter Corke's Robotics Toolbox
startup_rvc;

% Define the 6-DOF Robotic Manipulator using DH Parameters
L1 = Link('d', 1, 'a', 0, 'alpha', pi/2, 'standard');
L2 = Link('d', 0, 'a', 2, 'alpha', 0, 'standard');
L3 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'standard');
L4 = Link('d', 2, 'a', 0, 'alpha', pi/2, 'standard');
L5 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'standard');
L6 = Link('d', 1, 'a', 0, 'alpha', 0, 'standard');


% Create SerialLink Object
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6-DOF Manipulator');

% Define Start and Goal Positions in Cartesian Space (within reachable workspace)
P1 = [-0.2, -0.2, 0.2];  % Start position
P2 = [0.3, 0.3, 0.2];    % Goal position


% Define a Single Spherical Obstacle (in the way)
obs_center = [0.05, 0.05, 0.2];  % Between P1 and P2
obs_radius = 0.07;               % Reasonable radius


% Convert Cartesian Positions to Transformation Matrices
T1 = transl(P1) * trotx(pi);
T2 = transl(P2) * trotx(pi);

% Solve Inverse Kinematics for Start and Goal Positions
q0 = zeros(1,6);
q1 = robot.ikine(T1, q0, [1 1 1 1 1 1]);
q2 = robot.ikine(T2, q1, [1 1 1 1 1 1]);

if isempty(q1) || isempty(q2)
    error('Inverse kinematics failed. Adjust start and goal positions.');
end

% Generate Unoptimized Trajectory (Without PSO)
[q_unopt, ~, ~] = jtraj(q1, q2, linspace(0, 5, 100));

% Delete any previously saved best trajectory to force PSO to run
if exist('best_gbest.mat', 'file')
    delete('best_gbest.mat');
end
fprintf('Running PSO Optimization...\n');

% Define PSO Parameters with Randomized Values for Variation
numParticles = 100;
numIterations = 500;
w = 0.6 + 0.2 * rand; 
c1 = 1.5 + 0.5 * rand; 
c2 = 2.0 + 0.5 * rand; 

% Initialize Particles
particles = rand(numParticles, 6) * (max(q2) - min(q1)) + min(q1);
velocities = zeros(size(particles));
pbest = particles;
gbest = particles(1, :);
pbest_scores = inf(numParticles, 1);
gbest_score = inf;

% PSO Loop
for iter = 1:numIterations
    for i = 1:numParticles
        T_current = robot.fkine(particles(i, :));
        x_current = transl(T_current);
        
        % Compute fitness function (Distance + Obstacle Avoidance)
        dist_to_goal = norm(x_current - P2);
        obstacle_penalty = 0;
        d_obs = norm(x_current - obs_center);
        if d_obs < obs_radius * 1.5  
            obstacle_penalty = (10/d_obs^4); 
        end
        fitness = dist_to_goal + obstacle_penalty;
        
        % Update personal and global best
        if fitness < pbest_scores(i)
            pbest_scores(i) = fitness;
            pbest(i, :) = particles(i, :);
        end
        if fitness < gbest_score
            gbest_score = fitness;
            gbest = particles(i, :);
        end
    end
    
    % Update velocity and position of particles
    for i = 1:numParticles
        velocities(i, :) = w * velocities(i, :) + ...
                        c1 * rand * (pbest(i, :) - particles(i, :)) + ...
                        c2 * rand * (gbest - particles(i, :));
        particles(i, :) = particles(i, :) + velocities(i, :);
    end
end

% Save the best trajectory found
save('best_gbest.mat', 'gbest');
fprintf('Best trajectory saved for future runs.\n');

% Extract Optimized Trajectory (With PSO)
[q_opt, qd, qdd] = jtraj(q1, gbest, linspace(0, 5, 100));

% Compute trajectory path time (sum of Euclidean distances between points)
x_unopt = zeros(size(q_unopt,1), 3);
x_opt = zeros(size(q_opt,1), 3);
for i = 1:size(q_unopt,1)
    Ti_unopt = robot.fkine(q_unopt(i, :));
    x_unopt(i, :) = transl(Ti_unopt);
end
for i = 1:size(q_opt,1)
    Ti_opt = robot.fkine(q_opt(i, :));
    x_opt(i, :) = transl(Ti_opt);
end
traj_time_unopt = sum(sqrt(sum(diff(x_unopt).^2, 2))); % Without PSO
traj_time_opt = sum(sqrt(sum(diff(x_opt).^2, 2))); % With PSO

% Compute fitness function value for both paths
fitness_unopt = norm(x_unopt(end, :) - P2); % Distance to goal (without PSO)
fitness_opt = norm(x_opt(end, :) - P2); % Distance to goal (with PSO)

% Display the results
fprintf('Unoptimized Trajectory Path Time: %.4f seconds\n', traj_time_unopt);
fprintf('Optimized Trajectory Path Time: %.4f seconds\n', traj_time_opt);
fprintf('Unoptimized Trajectory Fitness Value: %.4f\n', fitness_unopt);
fprintf('Optimized Trajectory Fitness Value: %.4f\n', fitness_opt);

%% **1️⃣ Figure 1 - Simulate Robot Motion (PSO Optimized)**
figure;
robot.plot(q_opt, 'trail', 'k-');
title('Robot Motion with PSO Optimized Trajectory');

%% **2️⃣ Figure 2 - Compare Trajectories With and Without PSO**
figure;
hold on;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Comparison: PSO Optimized vs. Unoptimized Trajectory');

% Plot Start and End Points
scatter3(P1(1), P1(2), P1(3), 100, 'bo', 'filled'); 
scatter3(P2(1), P2(2), P2(3), 100, 'go', 'filled'); 
text(P1(1), P1(2), P1(3), ' Start', 'FontSize', 12, 'Color', 'b');
text(P2(1), P2(2), P2(3), ' End', 'FontSize', 12, 'Color', 'g');

% Plot Unoptimized Trajectory (Without PSO)
plot3(x_unopt(:,1), x_unopt(:,2), x_unopt(:,3), 'b--', 'LineWidth', 2); % Blue Dashed Line

% Plot Optimized Trajectory (With PSO)
plot3(x_opt(:,1), x_opt(:,2), x_opt(:,3), 'r-', 'LineWidth', 2); % Red Line

% Plot Obstacle
[xs, ys, zs] = sphere(20);
xs = obs_radius * xs + obs_center(1);
ys = obs_radius * ys + obs_center(2);
zs = obs_radius * zs + obs_center(3);
surf(xs, ys, zs, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

% Add Legend and Hold Off
h1 = plot3(x_unopt(:,1), x_unopt(:,2), x_unopt(:,3), 'b--', 'LineWidth', 2); % Unoptimized
h2 = plot3(x_opt(:,1), x_opt(:,2), x_opt(:,3), 'r-', 'LineWidth', 2);      
h3 = scatter3(P1(1), P1(2), P1(3), 100, 'bo', 'filled');                    % Start
h4 = scatter3(P2(1), P2(2), P2(3), 100, 'go', 'filled');                    % End
h5 = surf(xs, ys, zs, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Obstacle

legend([h1, h2, h3, h4, h5], ...
       {'Optimized Trajectory', 'Unoptimized Trajectory', 'Start Position', 'End Position', 'Obstacle'});

view(3);
hold off;
