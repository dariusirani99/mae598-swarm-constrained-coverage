%% Main Script
% Darius Irani
clc; clear all;

%% Defining Variables
% User specifies number of robots
numRobots = input('Enter the number of robots: ');

% gets user specified boundary coordinates from function
[boundaryX, boundaryY, obstacleCenters, obstacleRadii, patrolX, patrolY] = get_boundary();
boundaryX = [boundaryX; boundaryX(1)];
boundaryY = [boundaryY; boundaryY(1)];
patrolX = [patrolX; patrolX(1)];
patrolY = [patrolY; patrolY(1)];

% initialize random robot position in the room
robot_positions = get_robot_positions(boundaryX, boundaryY, obstacleCenters, obstacleRadii, numRobots);

% plot the inititial setup
plot_initial_setup(boundaryX, boundaryY, obstacleCenters, obstacleRadii, robot_positions);

%% Defining Functions
x = linspace(min(boundaryX), max(boundaryX), 200);
y = linspace(min(boundaryY), max(boundaryY), 200);
[X, Y] = meshgrid(x, y);

% Obstacle Repulsive Field
beta_sphere = struct([]);
for i = 1:length(obstacleRadii)
    % Obstacle repulsion field for each obstacle
    beta_sphere(i).obsfn = ((X - obstacleCenters(i, 1)).^2 + ...
                            (Y - obstacleCenters(i, 2)).^2) - ...
                            obstacleRadii(i)^2;
end
beta_prod = ones(size(beta_sphere(1).obsfn));
for i = 1:length(beta_sphere)
    beta_prod = beta_prod .* beta_sphere(i).obsfn;
end

% Patrol Region Attractive Field
patrol_center = mean([patrolX, patrolY]);
gamma = (x-patrol_center(1)).^2 + (y-patrol_center(2)).^2;

% Function to calculate the euclidean norm distance
distance_to_boundary = @(x, y) min(sqrt((x - X).^2 + (y - Y).^2));
gamma_grid = arrayfun(distance_to_boundary, X, Y).^2;

% Navigation function based only on obstacle repulsion
kappa = 3.5; % Controls sharpness
nav = gamma ./ (gamma.^kappa + beta_prod) .^ (1/kappa);


% Compute the gradients of the navigation function
[nav_dx, nav_dy] = gradient(nav, x(2) - x(1), y(2) - y(1));

% running simulation
run_simulation(10000, robot_positions, nav_dx, nav_dy, x, y, patrolX, patrolY, boundaryX, boundaryY, obstacleCenters, obstacleRadii)


