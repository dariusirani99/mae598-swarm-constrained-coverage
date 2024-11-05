%% Get Boundary Function
% Gets the boundary input from the user
% User draws a floorplan - REQUIRES IMAGE PROCESSING TOOLBOX
function [boundaryX, boundaryY] = get_boundary()
    figure;
    axis([-50 50 -50 50]); % Set axis limits for -50(meters) to 50(meters)
    title('Draw the boundary for the workspace');
    hold on;
    grid on;
    h = drawpolygon('Color', 'b', 'LineWidth', 2); % User-defined boundary
    boundaryPoints = h.Position; % Stores the (x, y) coordinates of the boundary
    plot(boundaryPoints(:,1), boundaryPoints(:,2), 'b-', 'LineWidth', 2);
    xlabel('X');
    ylabel('Y');
    axis equal;
    % storing boundary
    boundaryX = boundaryPoints(:,1);
    boundaryY = boundaryPoints(:,2);
end
