%% Get Robot Valid Initial Positions
% function to initialize robots in random positions within the boundary
function [robotPositions] = get_robot_positions(boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, unknownObstacleCenters, unknownObstacleRadii, numRobots)
    robotPositions = zeros(numRobots, 2);
    for i = 1:numRobots
        validPosition = false; % Is position valid? flag
    
        while ~validPosition
            % Generate a random position within the workspace range
            x = (max(boundaryX) - min(boundaryX)) * rand + min(boundaryX);
            y = (max(boundaryY) - min(boundaryY)) * rand + min(boundaryY);
            
            % Check if position is within the boundary polygon
            if inpolygon(x, y, boundaryX, boundaryY)
                % Check distance to known obstacles
                withinKnownObstacle = false;
                for j = 1:size(knownObstacleCenters, 1)
                    distToKnownObstacle = norm([x, y] - knownObstacleCenters(j, :));
                    if distToKnownObstacle < knownObstacleRadii(j)
                        withinKnownObstacle = true;
                        break; 
                    end
                end

                % Check distance to unknown obstacles
                withinUnknownObstacle = false;
                for j = 1:size(unknownObstacleCenters, 1)
                    distToUnknownObstacle = norm([x, y] - unknownObstacleCenters(j, :));
                    if distToUnknownObstacle < unknownObstacleRadii(j)
                        withinUnknownObstacle = true;
                        break;
                    end
                end
                
                % Accept position if outside all known and unknown obstacles
                if ~withinKnownObstacle && ~withinUnknownObstacle
                    robotPositions(i, :) = [x, y];
                    validPosition = true;
                end
            end
        end
    end
end