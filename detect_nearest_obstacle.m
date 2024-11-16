%% CONTROLLER FUNCTION - detect obstacle within sensing radius
function nearby_obstacle = detect_nearest_obstacle(xPos, yPos, unknownObstacleCenters, unknownObstacleRadii, sensing_radius)
    % Ensure unknownObstacleRadii is a column vector
    unknownObstacleRadii = unknownObstacleRadii(:);  % Convert to column vector if necessary

    if ~isempty(unknownObstacleCenters) && size(unknownObstacleCenters, 2) == 2
        % Calculate the distance to each obstacle center
        distances = sqrt((xPos - unknownObstacleCenters(:,1)).^2 + (yPos - unknownObstacleCenters(:,2)).^2);
        
        % Check if each obstacle is within the sensing range plus its radius
        within_range = distances <= (sensing_radius + unknownObstacleRadii);
        
        % Filter to only those within range
        if any(within_range)
            % Find the index of the closest obstacle within range
            [~, closest_idx] = min(distances(within_range));
            
            % Map the closest_idx back to the original index in obstacleCenters
            obstacle_indices = find(within_range);
            nearby_obstacle = obstacle_indices(closest_idx);
        else
            nearby_obstacle = [];  % No obstacles within sensing range
        end
    else
        nearby_obstacle = [];
    end
end