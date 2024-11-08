%% Simulation Loop Function for Robots
function run_simulation(time_steps, robot_positions, nav_dx, nav_dy, x, y, patrolX, patrolY, boundaryX, boundaryY, obstacleCenters, obstacleRadii)
    % Parameters for simulation
    dt = 0.1; % Time step
    numSteps = time_steps; % Number of simulation steps

    figure;
    hold on;
    plot(boundaryX, boundaryY, 'b-', 'LineWidth', 2); % Plot boundary
    plot(patrolX, patrolY, 'g-', 'LineWidth', 2); % Plot patrol region
    for j = 1:size(obstacleCenters, 1)
        viscircles(obstacleCenters(j, :), obstacleRadii(j), 'Color', 'r'); % Plot obstacles
    end
    xlabel('X');
    ylabel('Y');
    axis equal;
    title('Robot Movement with Patrol Region Attraction');
    grid on;

    % Simulation loop
    for t = 1:numSteps
        for i = 1:size(robot_positions, 1)
            % Current robot position
            xPos = robot_positions(i, 1);
            yPos = robot_positions(i, 2);

            % Find closest point on the grid to the robot's current position
            [~, xIdx] = min(abs(xPos - x));
            [~, yIdx] = min(abs(yPos - y));

            % Get the gradient direction at the robot's position
            dx = -nav_dx(yIdx, xIdx); % Move in the direction opposite the gradient
            dy = -nav_dy(yIdx, xIdx);

            % Normalize the movement direction if necessary
            normFactor = sqrt(dx^2 + dy^2);
            if normFactor > 0
                dx = dx / normFactor;
                dy = dy / normFactor;
            end

            % Update robot position
            newX = xPos + dx * dt;
            newY = yPos + dy * dt;

            % Check if the new position is within the boundary
            if inpolygon(newX, newY, boundaryX, boundaryY)
                robot_positions(i, 1) = newX;
                robot_positions(i, 2) = newY;
            end
        end

        % Plot updated robot positions
        plot(robot_positions(:, 1), robot_positions(:, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        pause(0.01); % Small delay for visualization
    end
    hold off;
end
