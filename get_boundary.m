%% Get Boundary Function
% Gets the boundary input from the user
% User draws a floorplan - REQUIRES IMAGE PROCESSING TOOLBOX
function [boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, unknownObstacleCenters, unknownObstacleRadii] = get_boundary()
    figure;
    axis([-50 50 -50 50]); % Set axis limits for -50(meters) to 50(meters)
    title('Draw the boundary for the workspace');
    hold on;
    grid on;
    h = drawpolygon('Color', 'b', 'LineWidth', 2); % User-defined boundary
    boundaryPoints = h.Position; % Stores the (x, y) coordinates of the boundary
    h.InteractionsAllowed = 'none';
    plot(boundaryPoints(:,1), boundaryPoints(:,2), 'b-', 'LineWidth', 2);
    xlabel('X');
    ylabel('Y');
    axis equal;

    % Storing boundary
    boundaryX = boundaryPoints(:,1);
    boundaryY = boundaryPoints(:,2);

    % Initialize known and unknown obstacle lists
    knownObstacleCenters = [];
    knownObstacleRadii = [];
    unknownObstacleCenters = [];
    unknownObstacleRadii = [];

    % Continuously add obstacles until the user presses Enter
    disp('Press Enter to add circular obstacles, or type "done" to finish.');

    while true
        % Prompt user for input to continue or exit
        choice_obstacles = input('Press Enter to add an obstacle, or type "done" to exit: ', 's');
        
        % Check if the user typed "done" to exit the loop
        if strcmpi(choice_obstacles, 'done')
            break;
        end

        % Ask if the obstacle is known or unknown
        obstacle_type = input('Is this obstacle "known" or "unknown"? ', 's');
        
        % Draw a circle interactively for the obstacle
        if strcmpi(obstacle_type, 'known')
            h_circle = drawcircle('Color', [1, 0.5, 0]); % Orange color for known obstacles
            knownObstacleCenters = [knownObstacleCenters; h_circle.Center];
            knownObstacleRadii = [knownObstacleRadii; h_circle.Radius];
        elseif strcmpi(obstacle_type, 'unknown')
            h_circle = drawcircle('Color', 'r'); % Red color for unknown obstacles
            unknownObstacleCenters = [unknownObstacleCenters; h_circle.Center];
            unknownObstacleRadii = [unknownObstacleRadii; h_circle.Radius];
        else
            disp('Invalid input. Please enter "known" or "unknown" for the obstacle type.');
            continue; % Skip this iteration if invalid input
        end
        
        % Lock circle interaction to prevent further modification
        h_circle.InteractionsAllowed = 'none';
    end
    close()
end
