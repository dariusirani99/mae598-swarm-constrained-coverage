%% CONTROLLER FUNCTION - detect neighbors within sensing range
function neighbors = detect_neighbors(robot_idx, robot_positions, sensing_radius)
    % Calculate distances to all other robots
    distances = abs(sqrt(sum((robot_positions - robot_positions(robot_idx, :)).^2, 2)));
    % Find indices of robots within sensing radius (excluding itself)
    neighbors = find(distances <= sensing_radius & distances > 0);
end