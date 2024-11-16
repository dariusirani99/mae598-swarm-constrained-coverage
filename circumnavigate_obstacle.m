%% %% CONTROLLER FUNCTION - move around obstacle
function [dx, dy] = circumnavigate_obstacle(xPos, yPos, unknownObstacleCenter, unknownObstacleRadius)
    % Calculate direction to circumnavigate an obstacle
    to_center = unknownObstacleCenter - [xPos, yPos];
    tangent_vector = [-to_center(2), to_center(1)];
    norm_factor = norm(tangent_vector);
    dx = tangent_vector(1) / norm_factor;
    dy = tangent_vector(2) / norm_factor;
end
