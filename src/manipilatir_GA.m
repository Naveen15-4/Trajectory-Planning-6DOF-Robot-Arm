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

% Define Start and Goal Pose in Cartesian Space
P1 = [-0.180, 0, -0.114];  % Start Position (X, Y, Z)
P2 = [0.153, -0.10, -0.101];  % Goal Position (X, Y, Z)

% Define a Single Spherical Obstacle
obs_center = [0.0, -0.05, -0.20]; % Obstacle center
obs_radius = 0.10; % Obstacle radius

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

% Generate Unoptimized Trajectory (Without GA)
[q_unopt, ~, ~] = jtraj(q1, q2, linspace(0, 5, 100));

% Delete any previously saved best trajectory to force GA to run
if exist('best_gbest.mat', 'file')
    delete('best_gbest.mat');
end
fprintf('Running GA Optimization...\n');

% Define GA Parameters
populationSize = 100;
numGenerations = 500;
mutationRate = 0.1;
crossoverRate = 0.8;

% Initialize Population
population = rand(populationSize, 6) * (max(q2) - min(q1)) + min(q1);

% GA Loop
for generation = 1:numGenerations
    % Evaluate Fitness
    fitness = zeros(populationSize, 1);
    for i = 1:populationSize
        T_current = robot.fkine(population(i, :));
        x_current = transl(T_current);
        
        % Compute fitness function (Distance + Obstacle Avoidance)
        dist_to_goal = norm(x_current - P2);
        obstacle_penalty = 0;
        d_obs = norm(x_current - obs_center);
        if d_obs < obs_radius * 1.5  
            obstacle_penalty = (10/d_obs^4); 
        end
        fitness(i) = dist_to_goal + obstacle_penalty;
    end
    
    % Selection (Tournament Selection)
    newPopulation = zeros(size(population));
    for i = 1:populationSize
        % Select two random individuals
        idx1 = randi(populationSize);
        idx2 = randi(populationSize);
        % Select the one with better fitness
        if fitness(idx1) < fitness(idx2)
            newPopulation(i, :) = population(idx1, :);
        else
            newPopulation(i, :) = population(idx2, :);
        end
    end
    
    % Crossover (Single Point Crossover)
    for i = 1:2:populationSize
        if rand < crossoverRate
            crossoverPoint = randi(5);
            temp = newPopulation(i, crossoverPoint+1:end);
            newPopulation(i, crossoverPoint+1:end) = newPopulation(i+1, crossoverPoint+1:end);
            newPopulation(i+1, crossoverPoint+1:end) = temp;
        end
    end
    
    % Mutation
    for i = 1:populationSize
        if rand < mutationRate
            mutationPoint = randi(6);
            newPopulation(i, mutationPoint) = rand * (max(q2) - min(q1)) + min(q1);
        end
    end
    
    % Update Population
    population = newPopulation;
end

% Find the best individual
[~, bestIdx] = min(fitness);
gbest = population(bestIdx, :);

% Save the best trajectory found
save('best_gbest.mat', 'gbest');
fprintf('Best trajectory saved for future runs.\n');

% Extract Optimized Trajectory (With GA)
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
traj_time_unopt = sum(sqrt(sum(diff(x_unopt).^2, 2))); % Without GA
traj_time_opt = sum(sqrt(sum(diff(x_opt).^2, 2))); % With GA

% Compute fitness function value for both paths
fitness_unopt = norm(x_unopt(end, :) - P2); % Distance to goal (without GA)
fitness_opt = norm(x_opt(end, :) - P2); % Distance to goal (with GA)

% Display the results
fprintf('Unoptimized Trajectory Path Time: %.4f seconds\n', traj_time_unopt);
fprintf('Optimized Trajectory Path Time: %.4f seconds\n', traj_time_opt);
fprintf('Unoptimized Trajectory Fitness Value: %.4f\n', fitness_unopt);
fprintf('Optimized Trajectory Fitness Value: %.4f\n', fitness_opt);

%% **1️⃣ Figure 1 - Simulate Robot Motion (GA Optimized)**
figure;
robot.plot(q_opt, 'trail', 'k-');
title('Robot Motion with GA Optimized Trajectory');

%% **2️⃣ Figure 2 - Compare Trajectories With and Without GA**
figure;
hold on;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Comparison: GA Optimized vs. Unoptimized Trajectory');

% Plot Start and End Points
scatter3(P1(1), P1(2), P1(3), 100, 'bo', 'filled'); 
scatter3(P2(1), P2(2), P2(3), 100, 'go', 'filled'); 
text(P1(1), P1(2), P1(3), ' Start', 'FontSize', 12, 'Color', 'b');
text(P2(1), P2(2), P2(3), ' End', 'FontSize', 12, 'Color', 'g');

% Plot Unoptimized Trajectory (Without GA)
plot3(x_unopt(:,1), x_unopt(:,2), x_unopt(:,3), 'b--', 'LineWidth', 2); % Blue Dashed Line

% Plot Optimized Trajectory (With GA)
plot3(x_opt(:,1), x_opt(:,2), x_opt(:,3), 'r-', 'LineWidth', 2); % Red Line

% Plot Obstacle
[xs, ys, zs] = sphere(20);
xs = obs_radius * xs + obs_center(1);
ys = obs_radius * ys + obs_center(2);
zs = obs_radius * zs + obs_center(3);
surf(xs, ys, zs, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');

% Add Legend and Hold Off
h1 = plot3(x_unopt(:,1), x_unopt(:,2), x_unopt(:,3), 'b--', 'LineWidth', 2); % Unoptimized
h2 = plot3(x_opt(:,1), x_opt(:,2), x_opt(:,3), 'r-', 'LineWidth', 2);       % Optimized
h3 = scatter3(P1(1), P1(2), P1(3), 100, 'bo', 'filled');                    % Start
h4 = scatter3(P2(1), P2(2), P2(3), 100, 'go', 'filled');                    % End
h5 = surf(xs, ys, zs, 'FaceColor', 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none'); % Obstacle

legend([h1, h2, h3, h4, h5], ...
       {'Unoptimized Trajectory', 'Optimized Trajectory', 'Start Position', 'End Position', 'Obstacle'});

view(3);
hold off;