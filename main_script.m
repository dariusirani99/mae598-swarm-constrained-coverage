%% Main Script
% Darius Irani
clc; clear all;


%% START OF SCRIPT
% User specifies number of robots
numRobots = input('Enter the number of robots: ');

%% USER TUNABLE VARIABLES
% Defining robot states and switching probabilities
p_leave_base = 0.10;  % Base probability of leaving a cluster
sensing_radius = 1.5; % from robot below
num_points_per_segment = 50;
robot_speed = 0.25; % Base speed of random movement should equal .25m/s (https://www.lotsofbots.com/en/l1000-versus-unmanned-ground-vehicle-ugv/)

%% Get initial variables and states
% Robot states: 0 for searching, 1 for mapping
robot_states = zeros(numRobots, 1);  % Initially, all are searching

% gets user specified boundary coordinates from function
[boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, unknownObstacleCenters, unknownObstacleRadii] = get_boundary();
boundaryX = [boundaryX; boundaryX(1)];
boundaryY = [boundaryY; boundaryY(1)];

% initialize random robot position in the room
robot_positions = get_robot_positions(boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, ...
    unknownObstacleCenters, unknownObstacleRadii, numRobots);

% plot the inititial setup
figure(1);
clf; % Clear any existing content in figure 1
plot_initial_setup(boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, unknownObstacleCenters, unknownObstacleRadii, robot_positions);
title('Initial Setup');
xlabel('X');
ylabel('Y');
axis equal;

%% Navigation Function (Obstacle repulsion)
x = linspace(min(boundaryX), max(boundaryX), 200);
y = linspace(min(boundaryY), max(boundaryY), 200);
[X, Y] = meshgrid(x, y);
kappa = 3.5; % controls sharpness of nav

% Obstacle Repulsive Field
beta_sphere = struct([]);
if isempty(knownObstacleCenters)
    beta_prod = ones(size(X));
else
    for i = 1:length(knownObstacleRadii)
        % Obstacle repulsion field for each obstacle
        beta_sphere(i).obsfn = ((X - knownObstacleCenters(i, 1)).^2 + ...
                                (Y - knownObstacleCenters(i, 2)).^2) - ...
                                knownObstacleRadii(i)^2;
    end
    
    % Compute product of all obstacle functions
    beta_prod = ones(size(beta_sphere(1).obsfn));
    for i = 1:length(beta_sphere)
        beta_prod = beta_prod .* beta_sphere(i).obsfn;
    end
end

% scale and threshold the navigation function
nav = 1 ./ ((beta_prod) .^ (1 / kappa));

% Step 1: Create a distance mask to limit the nav function to regions near known obstacles
distance_mask = false(size(X)); % Initialize the mask
for i = 1:length(knownObstacleRadii)
    obstacle_radius = knownObstacleRadii(i);
    % Calculate the squared distance from each grid point to the obstacle center
    distances_squared = (X - knownObstacleCenters(i, 1)).^2 + (Y - knownObstacleCenters(i, 2)).^2;
    % Update the mask for points within the obstacles radius outside of the
    % obstacle
    distance_mask = distance_mask | (sqrt(distances_squared) <= (knownObstacleRadii(i) + obstacle_radius/2));
end
% Set all navigation values outside the mask to zero
nav(~distance_mask) = 0;

% Smoothly taper values at the boundary of the mask for each obstacle
for i = 1:length(knownObstacleRadii)
    obstacle_radius = knownObstacleRadii(i);
    % Calculate the squared distance from each grid point to the obstacle center
    distances_squared = (X - knownObstacleCenters(i, 1)).^2 + (Y - knownObstacleCenters(i, 2)).^2;
    % Create a mask for points near this specific obstacle within its
    % radius outside of the obstacle
    obstacle_mask = sqrt(distances_squared) <= (knownObstacleRadii(i) + obstacle_radius/2);
    
    % Extract non-zero navigation values within the obstacle mask
    nav_obstacle = nav(obstacle_mask); % Values within this obstacle's region
    if any(nav_obstacle > 0)
        % Find the minimum non-zero value for this obstacle
        min_value = min(nav_obstacle(nav_obstacle > 0));
        
        % Shift the navigation function within the mask
        nav(obstacle_mask) = nav(obstacle_mask) - min_value;
        % Ensure no negative values within the mask
        nav(nav < 0) = 0;

        % Smoothly taper values at the boundary of the mask
        distances_within_mask = sqrt(distances_squared(obstacle_mask));
        max_distance = knownObstacleRadii(i) + obstacle_radius/2;
        taper_factor = (max_distance - distances_within_mask) / max_distance;
        taper_factor = max(taper_factor, 0); % taper_factor is non-negative
        nav(obstacle_mask) = nav(obstacle_mask) .* taper_factor;
    end
end

% visualize the field
figure(2); % Navigation field plot
clf;
visualize_repulsive_field(X, Y, real(nav));

% Compute the gradients of the navigation function and scale
[nav_dx, nav_dy] = gradient(nav, x(2) - x(1), y(2) - y(1));

% interpolate boundary points
[interpBoundaryX, interpBoundaryY] = interpolate_boundary(boundaryX, boundaryY, num_points_per_segment);

%% Running simulation for number of seconds
figure(3);
clf;
run_simulation(10000, robot_positions, nav_dx, nav_dy, x, y, interpBoundaryX, interpBoundaryY, knownObstacleCenters, knownObstacleRadii, ...
    unknownObstacleCenters, unknownObstacleRadii, sensing_radius, robot_states, p_leave_base, robot_speed, numRobots)
