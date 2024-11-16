%% NOT CONTROLLER FUNCTION - BASIC LOGIG - check if robot is in restricted unknown obstacle
function [withinUnknownObstacle] = check_inside_obstacle(xPos, yPos, unknownObstacleCenters, unknownObstacleRadii, sensing_radius)
    withinUnknownObstacle = false; % Initialize as not inside any obstacle
    tolerance = 1; % Allow for numerical imprecision

    for j = 1:size(unknownObstacleCenters, 1)
        % Extract obstacle center
        obstacleCenter = unknownObstacleCenters(j, :);
        
        % Calculate Euclidean distance
        distToUnknownObstacle = norm([xPos, yPos] - obstacleCenter);
        
        % Calculate expanded radius (obstacle radius + sensing radius)
        expandedRadius = unknownObstacleRadii(j) + (sensing_radius - tolerance);

        % Check if within the expanded radius (with tolerance)
        if distToUnknownObstacle < expandedRadius
            withinUnknownObstacle = true;
            break;
        end
    end
end
