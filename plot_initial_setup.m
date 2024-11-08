%% Plot Initial Configuration
% function to initialize robots in random positions within the boundary
% Plot the initial setup
function plot_initial_setup(boundaryX, boundaryY, obstacleCenters, obstacleRadii, robotPositions)
    % Plot the initial configuration of the workspace, obstacles, and robots
    figure;
    plot(boundaryX, boundaryY, 'b-', 'LineWidth', 2); % Plot boundary
    hold on;
    
    % Plot obstacles
    for j = 1:size(obstacleCenters, 1)
        viscircles(obstacleCenters(j, :), obstacleRadii(j), 'Color', 'r');
    end
    
    % Plot robots
    plot(robotPositions(:,1), robotPositions(:,2), 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
    xlabel('X');
    ylabel('Y');
    title('Initial Robot Positions within Workspace');
    axis equal;
    grid on;
    hold off;
end