function run_simulation(time_steps, robot_positions, nav_dx, nav_dy, x, y, boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, ...
    unknownObstacleCenters, unknownObstacleRadii, sensing_radius, p_encounter_base, p_leave_base)
    % Parameters for simulation
    dt = 0.1; % Time step
    base_speed = 0.75; % Base speed of random movement

    % Initial random directions for each robot
    direction_angles = 2 * pi * rand(size(robot_positions, 1), 1);
 
    % Plot setup
    figure;
    hold on;
    plot(boundaryX, boundaryY, 'b-', 'LineWidth', 2); % Plot boundary
    if ~isempty(knownObstacleCenters)
        for j = 1:size(knownObstacleCenters, 1)
            viscircles(knownObstacleCenters(j, :), knownObstacleRadii(j), 'Color', [1, 0.5, 0]); % Plot known obstacles
        end
    end
    xlabel('X');
    ylabel('Y');
    axis equal;
    title('Robot Movement with Obstacle Avoidance and Boundary Reflection');
    grid on;

    % Initialize plot for robot positions without trails
    h_robots = plot(robot_positions(:, 1), robot_positions(:, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

    % Simulation loop
    for t = 1:time_steps
        for i = 1:size(robot_positions, 1)
            % Current robot position
            xPos = robot_positions(i, 1);
            yPos = robot_positions(i, 2);

            % Boundary reflection
            if abs(min(sqrt((boundaryX - xPos).^2 + (boundaryY - yPos).^2))) < sensing_radius
                [xPos, yPos, direction_angles(i)] = apply_boundary_reflection(xPos, yPos, boundaryX, boundaryY, sensing_radius, direction_angles(i));
            end
            % Random movement
            dx = cos(direction_angles(i)) * base_speed;
            dy = sin(direction_angles(i)) * base_speed;
            

            % Apply obstacle avoidance
            [repulsive_dx, repulsive_dy] = compute_obstacle_repulsion(xPos, yPos, nav_dx, nav_dy, x, y);
            dx = dx + repulsive_dx;
            dy = dy + repulsive_dy;

            % Normalize the movement vector
            movement_norm = sqrt(dx^2 + dy^2);
            if movement_norm > 0
                dx = dx / movement_norm;
                dy = dy / movement_norm;
            end

            % Predict new position
            xPos = xPos + dx * dt;
            yPos = yPos + dy * dt;

            % Update robot position
            robot_positions(i, 1) = xPos;
            robot_positions(i, 2) = yPos;
        end
        
        

        % Update robot positions on the plot
        set(h_robots, 'XData', robot_positions(:, 1), 'YData', robot_positions(:, 2));
        pause(0.01); % Small delay for visualization
    end
    hold off;
end
