%% CONTROLLER FUNCTION - move around obstacle
function [dx, dy] = circumnavigate_obstacle(xPos, yPos, obstacle_center, obstacle_radius, sensing_radius, base_speed)
    % Calculate vector from robot to obstacle center
    to_center_x = obstacle_center(1) - xPos;
    to_center_y = obstacle_center(2) - yPos;

    % Calculate the current distance from the obstacle center
    current_distance = sqrt(to_center_x^2 + to_center_y^2);

    % Desired distance is obstacle radius + sensing radius
    desired_distance = obstacle_radius + sensing_radius;

    % Adjust position to maintain the desired distance
    distance_adjustment_factor = (desired_distance - current_distance) / current_distance;
    adjusted_x = xPos + to_center_x * distance_adjustment_factor;
    adjusted_y = yPos + to_center_y * distance_adjustment_factor;

    % Recalculate the vector from the adjusted position to the obstacle center
    to_center_x = obstacle_center(1) - adjusted_x;
    to_center_y = obstacle_center(2) - adjusted_y;

    % Normalize vector
    norm = sqrt(to_center_x^2 + to_center_y^2);
    if norm == 0
        norm = 1; % Prevent division by zero
    end
    to_center_x = to_center_x / norm;
    to_center_y = to_center_y / norm;

    % Calculate tangential direction (90 degrees from vector to center)
    dx = -to_center_y;
    dy = to_center_x;

end
