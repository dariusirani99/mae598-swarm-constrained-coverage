%% Get Boundary Function
% Gets the boundary input from the user
% User draws a floorplan - REQUIRES IMAGE PROCESSING TOOLBOX
function [boundaryX, boundaryY, obstacleCenters, obstacleRadii, patrolX, patrolY] = get_boundary()
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
    % storing boundary
    boundaryX = boundaryPoints(:,1);
    boundaryY = boundaryPoints(:,2);
    % allowing users to define circular obstacles
    obstacleCenters = [];
    obstacleRadii = [];
    % Continuously add obstacles until the user presses Enter
    disp('Press Enter to add circular obstacles, or type "done" to finish.');
    obstacleCenters = [];
    obstacleRadii = [];
    
    % Continuously add obstacles until the user decides to stop
    while true
        % Prompt user for input to continue or exit
        choice_obstacles = input('Press Enter to add obstacles, or type "done" to exit: ', 's');
        
        % Check if the user typed "done" to exit the loop
        if strcmpi(choice_obstacles, 'done')
            break;
        end
        
        % Draw a circle interactively, allowing the user to place and size it
        h_circle = drawcircle('Color', 'r'); % User defines both center and radius interactively

        % Store the obstacle's center and radius
        obstacleCenters = [obstacleCenters; h_circle.Center];
        obstacleRadii = [obstacleRadii; h_circle.Radius];
        
        % Lock circle interaction to prevent further modification
        h_circle.InteractionsAllowed = 'none';
    end
    % Define patrol region
    disp('Please draw the patrol region of the robots:');
    h_patrol = drawpolygon('Color', 'g', 'LineWidth', 2); % User-defined patrol region
    patrolPoints = h_patrol.Position; % Stores patrol region points
    patrolX = patrolPoints(:,1);
    patrolY = patrolPoints(:,2);
    plot(patrolX, patrolY, 'g-', 'LineWidth', 2);
    hold off;
    close();
end
