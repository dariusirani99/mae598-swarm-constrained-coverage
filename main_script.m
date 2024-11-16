%% Main Script
% Darius Irani
clc; clear all;


%% Defining Variables
% User specifies number of robots
numRobots = input('Enter the number of robots: ');

% Defining robot states and switching probabilities
p_encounter_base = 0.05;  % Base probability of joining a cluster
p_leave_base = 0.1;  % Base probability of leaving a cluster

% Robot states: 0 for searching, 1 for aggregated
robot_states = zeros(numRobots, 1);  % Initially, all are searching
robot_clusters = zeros(numRobots, 1);  % Cluster assignment
sensing_radius = 2;

% gets user specified boundary coordinates from function
[boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, unknownObstacleCenters, unknownObstacleRadii] = get_boundary();
boundaryX = [boundaryX; boundaryX(1)];
boundaryY = [boundaryY; boundaryY(1)];

% initialize random robot position in the room
robot_positions = get_robot_positions(boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, ...
    unknownObstacleCenters, unknownObstacleRadii, numRobots);

% plot the inititial setup
plot_initial_setup(boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, unknownObstacleCenters, unknownObstacleRadii, robot_positions);

%% Defining Functions
x = linspace(min(boundaryX), max(boundaryX), 200);
y = linspace(min(boundaryY), max(boundaryY), 200);
[X, Y] = meshgrid(x, y);

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

epsilon = 1e-6;
beta_prod = beta_prod + epsilon;

% Combined Navigation Function (Combining boundary and obstacle repulsion)
kappa = 5; % Controls repulsion sharpness
nav = 1 ./ (beta_prod .^ (1 / kappa));
%visualize_repulsive_field(X, Y, nav);

% Compute the gradients of the navigation function
[nav_dx, nav_dy] = gradient(nav, x(2) - x(1), y(2) - y(1));

% interpolate boundary points
num_points_per_segment = 50;
[interpBoundaryX, interpBoundaryY] = interpolate_boundary(boundaryX, boundaryY, num_points_per_segment);

%% Running simulation for number of seconds
run_simulation(10000, robot_positions, nav_dx, nav_dy, x, y, interpBoundaryX, interpBoundaryY, knownObstacleCenters, knownObstacleRadii, ...
    unknownObstacleCenters, unknownObstacleRadii, sensing_radius, p_encounter_base, p_leave_base)
