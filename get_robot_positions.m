%% Get Robot Valid Initial Positions
% function to initialize robots in random positions within the boundary
function [robotPositions] = get_robot_positions(boundaryX, boundaryY, obstacleCenters, obstacleRadii, numRobots)
    robotPositions = zeros(numRobots, 2);
    for i = 1:numRobots
        validPosition = false; % is position valid? flag
    
        while ~validPosition
            % random position within the workspace range
            x = (max(boundaryX) - min(boundaryX)) * rand + min(boundaryX);
            y = (max(boundaryY) - min(boundaryY)) * rand + min(boundaryY);
            
            % position is within the boundary polygon
            if inpolygon(x, y, boundaryX, boundaryY)
                % Check distance to all obstacles
                withinObstacle = false;
                for j = 1:size(obstacleCenters, 1)
                    distToObstacle = norm([x, y] - obstacleCenters(j, :));
                    if distToObstacle < obstacleRadii(j)
                        withinObstacle = true;
                        break; 
                    end
                end
                
                % If position is outside all obstacles, accept it
                if ~withinObstacle
                    robotPositions(i, :) = [x, y];
                    validPosition = true;
                end
            end
        end
    end
end