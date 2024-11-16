%% Plot Initial Configuration
% function to initialize robots in random positions within the boundary
% Plot the initial setup
function plot_initial_setup(boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, unknownObstacleCenters, unknownObstacleRadii, robotPositions)
    % Plot the initial configuration of the workspace, obstacles, and robots
    plot(boundaryX, boundaryY, 'b-', 'LineWidth', 2); % Plot boundary
    hold on;
    % Plot known obstacles in orange
    for j = 1:size(knownObstacleCenters, 1)
        viscircles(knownObstacleCenters(j, :), knownObstacleRadii(j), 'Color', [1, 0.5, 0]); % Orange color for known obstacles
    end

    % Plot unknown obstacles in red
    for j = 1:size(unknownObstacleCenters, 1)
        viscircles(unknownObstacleCenters(j, :), unknownObstacleRadii(j), 'Color', 'r'); % Red color for unknown obstacles
    end
    
    % Plot robots in green
    plot(robotPositions(:,1), robotPositions(:,2), 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
    
    xlabel('X');
    ylabel('Y');
    title('Initial Robot Positions within Workspace');
    axis equal;
    grid on;
end