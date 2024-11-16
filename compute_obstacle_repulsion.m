%% Function for robot boundary repulsion forces
function [repulsive_dx, repulsive_dy] = compute_obstacle_repulsion(xPos, yPos, nav_dx, nav_dy, x, y)
    % Find the closest grid point for the robot position
    [~, xIdx] = min(abs(xPos - x));
    [~, yIdx] = min(abs(yPos - y));

    % Retrieve the navigation gradient at that position
    repulsive_dx = -nav_dx(yIdx, xIdx);
    repulsive_dy = -nav_dy(yIdx, xIdx);

    % Normalize the repulsive force to ensure bounded effect
    repulsion_norm = sqrt(repulsive_dx^2 + repulsive_dy^2);
    if repulsion_norm > 0
        repulsive_dx = repulsive_dx / repulsion_norm;
        repulsive_dy = repulsive_dy / repulsion_norm;
    end
end
