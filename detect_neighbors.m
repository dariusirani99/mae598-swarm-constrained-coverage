%% CONTROLLER FUNCTION - detect neighbors within sensing range
function neighbors = detect_neighbors(robot_idx, robot_positions, sensing_radius)
    % Identify neighbors within sensing range
    distances = sqrt(sum((robot_positions - robot_positions(robot_idx, :)).^2, 2));
    neighbors = find(distances < sensing_radius & distances > 0);
end