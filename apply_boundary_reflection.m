%% CONTROLLER FUNCTION - reflects across boundaries
function [xPos, yPos, direction_angle] = apply_boundary_reflection(xPos, yPos, boundaryX, boundaryY, sensing_radius, direction_angle)
    % Calculate the distance to the boundary
    distances_to_boundary = sqrt((boundaryX - xPos).^2 + (boundaryY - yPos).^2);
    [min_dist, closest_idx] = min(distances_to_boundary);

    if min_dist < sensing_radius

        % Get the closest boundary point
        closest_boundary_x = boundaryX(closest_idx);
        closest_boundary_y = boundaryY(closest_idx);

        % Compute the normal vector of the boundary at the closest point
        normal_x = xPos - closest_boundary_x;
        normal_y = yPos - closest_boundary_y;

        % Normalize the normal vector
        normal_norm = sqrt(normal_x^2 + normal_y^2);
        if normal_norm > 0
            normal_x = normal_x / normal_norm;
            normal_y = normal_y / normal_norm;
        end

        % Adjust the direction to move away from the boundary
        incident_dx = cos(direction_angle);
        incident_dy = sin(direction_angle);

        % Reflect the incident vector across the boundary normal
        dot_product = 2 * (incident_dx * normal_x + incident_dy * normal_y);
        reflected_dx = incident_dx - dot_product * normal_x;
        reflected_dy = incident_dy - dot_product * normal_y;

        % Check the angle and ensure it moves away from the boundary
        angle_to_normal = atan2(reflected_dy, reflected_dx);

        % Add some randomness to the reflection to ensure smoother motion
        randomness_factor = (rand() - 0.5) * pi / 4; % Random adjustment of +/- 22.5 degrees
        direction_angle = angle_to_normal + randomness_factor;

        % Move the robot slightly inside the boundary to avoid re-triggering
        xPos = closest_boundary_x + sensing_radius * normal_x;
        yPos = closest_boundary_y + sensing_radius * normal_y;
    end
end