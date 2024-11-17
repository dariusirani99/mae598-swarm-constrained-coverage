%% Interpolate boundary points
function [interpBoundaryX, interpBoundaryY] = interpolate_boundary(boundaryX, boundaryY, num_points_per_segment)
    % arrays for interpolated boundary points
    interpBoundaryX = [];
    interpBoundaryY = [];

    for i = 1:length(boundaryX) - 1
        % Get the start and end points of the segment
        x_start = boundaryX(i);
        y_start = boundaryY(i);
        x_end = boundaryX(i + 1);
        y_end = boundaryY(i + 1);

        % Interpolate points along the segment
        interpX = linspace(x_start, x_end, num_points_per_segment);
        interpY = linspace(y_start, y_end, num_points_per_segment);

        % Append the interpolated points to the list
        interpBoundaryX = [interpBoundaryX, interpX];
        interpBoundaryY = [interpBoundaryY, interpY];
    end
end