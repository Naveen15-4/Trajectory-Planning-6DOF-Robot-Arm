clc; clear; close all;
startup_rvc;

%% Step 1: Define 6-DOF Robot (Puma 560 Example)
mdl_puma560; % Load Puma 560 model
robot = p560;

% Start and Goal Joint Configurations
q_start = [0 0 0 0 0 0]; 
q_goal = [pi/2 -pi/4 pi/4 -pi/6 pi/3 -pi/4]; 

%% Step 2: Define the Obstacle (3D Sphere)
obstacle_center = [0.5, 0.3, 0.2]; % (x, y, z)
obstacle_radius = 0.1;

% Plot the obstacle (3D Sphere)
[x, y, z] = sphere(20);
x = x * obstacle_radius + obstacle_center(1);
y = y * obstacle_radius + obstacle_center(2);
z = z * obstacle_radius + obstacle_center(3);

figure;
hold on;
surf(x, y, z, 'FaceColor', 'r', 'EdgeColor', 'none'); 
alpha(0.6); % Transparency
xlabel('X'); ylabel('Y'); zlabel('Z');
grid on; axis equal; hold on;

%% Step 3: PSO Parameters
num_particles = 10;
num_iterations = 20;
num_joints = 6;
num_points = 20; % Number of waypoints in trajectory
w = 0.7; c1 = 1.5; c2 = 1.5; % PSO tuning parameters

% Initialize particles (random joint trajectories)
particles = rand(num_particles, num_points, num_joints) * 2 * pi - pi;
velocities = zeros(num_particles, num_points, num_joints);
pBest = particles; % Personal best positions
fitness = inf(num_particles, 1);
gBest = particles(1, :, :); % Global best trajectory

%% Step 4: Fitness Function Definition
for i = 1:num_particles
    fitness(i) = compute_fitness(robot, squeeze(particles(i, :, :)), q_goal, obstacle_center, obstacle_radius);
    if fitness(i) < compute_fitness(robot, squeeze(gBest), q_goal, obstacle_center, obstacle_radius)
        gBest = particles(i, :, :);
    end
end

%% Step 5: PSO Main Loop
for iter = 1:num_iterations
    for i = 1:num_particles
        % Update velocity and position
        velocities(i, :, :) = w * velocities(i, :, :) + ...
            c1 * rand * (pBest(i, :, :) - particles(i, :, :)) + ...
            c2 * rand * (gBest - particles(i, :, :));
        
        particles(i, :, :) = particles(i, :, :) + velocities(i, :, :);
        
        % Compute new fitness
        new_fitness = compute_fitness(robot, squeeze(particles(i, :, :)), q_goal, obstacle_center, obstacle_radius);
        
        % Update personal and global bests
        if new_fitness < fitness(i)
            fitness(i) = new_fitness;
            pBest(i, :, :) = particles(i, :, :);
        end
        if new_fitness < compute_fitness(robot, squeeze(gBest), q_goal, obstacle_center, obstacle_radius)
            gBest = particles(i, :, :);
        end
    end
end

%% Step 6: Execute the Optimized Collision-Free Trajectory
% Plot only the best path
robot.plot(squeeze(gBest), 'delay', 0.5, 'trail', 'r-');
title('6-DOF Manipulator with PSO Avoiding a 3D Spherical Obstacle');

%% Function: Compute Fitness of a Trajectory
function cost = compute_fitness(robot, q_traj, q_goal, obstacle_center, obstacle_radius)
    num_points = size(q_traj, 1);
    cost = 0;

    for i = 1:num_points
        % Extract the i-th joint configuration (ensure it is 1x6)
        q = q_traj(i, :); % This should be 1x6
        q = q(:)'; % Ensure it is a row vector

        % Forward kinematics
        T = robot.fkine(q); % Compute end-effector pose
        ee_pos = T.t'; % End-effector position

        % Distance to goal (minimize)
        goal_dist = norm(ee_pos - robot.fkine(q_goal).t');

        % Collision penalty (high penalty if inside obstacle)
        obstacle_dist = norm(ee_pos - obstacle_center);
        if obstacle_dist < obstacle_radius
            collision_penalty = 1000; % High penalty for collision
        else
            collision_penalty = 0;
        end

        % Joint smoothness penalty (avoid sudden changes)
        if i > 1
            q_prev = q_traj(i-1, :); % Previous joint configuration
            q_prev = q_prev(:)'; % Ensure it is a row vector
            joint_smoothness = norm(q - q_prev);
        else
            joint_smoothness = 0;
        end

        % Total fitness function
        cost = cost + goal_dist + collision_penalty + 0.1 * joint_smoothness;
    end
end