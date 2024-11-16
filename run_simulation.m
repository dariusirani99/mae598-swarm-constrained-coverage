function run_simulation(time_steps, robot_positions, nav_dx, nav_dy, x, y, boundaryX, boundaryY, knownObstacleCenters, knownObstacleRadii, ...
    unknownObstacleCenters, unknownObstacleRadii, sensing_radius, robot_states, p_leave_base, base_speed, numRobots)
    % Parameters for simulation
    dt = 0.1; % Time step
    base_speed = dt * base_speed; % scaling speed with dt
    robot_states = robot_states;  % Initial states should be searching
    p_leave_base = dt * p_leave_base; % probability scaled by timestep

    % Initial random directions for each robot
    direction_angles = 2 * pi * rand(size(robot_positions, 1), 1);
    % Initialize state change times
    state_change_time = zeros(numRobots, 1);

    % Plot setup
    figure(3);
    clf;
    plot(boundaryX, boundaryY, 'b-', 'LineWidth', 2); % Plot boundary
    if ~isempty(knownObstacleCenters)
        for j = 1:size(knownObstacleCenters, 1)
            viscircles(knownObstacleCenters(j, :), knownObstacleRadii(j), 'Color', [1, 0.5, 0]); % Known obstacles
        end
    end
    xlabel('X');
    ylabel('Y');
    axis equal;
    title('Robot Movement');
    grid on;
    hold on;

    % Initialize plot for robot positions without trails
    h_robots = plot(robot_positions(:, 1), robot_positions(:, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    hold on;

    % Simulation loop
    for t = 1:time_steps
        elapsed_time = t * dt;
        for i = 1:size(robot_positions, 1)
            % Current robot position
            xPos = robot_positions(i, 1);
            yPos = robot_positions(i, 2);

            % Boundary reflection
            if abs(min(sqrt((boundaryX - xPos).^2 + (boundaryY - yPos).^2))) < sensing_radius
                [xPos, yPos, direction_angles(i)] = apply_boundary_reflection(xPos, yPos, boundaryX, boundaryY, sensing_radius, direction_angles(i));
            end
            
            % communicating states with neighbors within sensing radius
            neighbors = detect_neighbors(i, robot_positions, sensing_radius);
            neighbor_states = robot_states(neighbors);
            num_mapping_neighbors = sum(neighbor_states == 1);
            
            % time since last state change
            time_since_last_change = elapsed_time - state_change_time(i);

            % Early exit for searching case
            if robot_states(i) == 0 && time_since_last_change > 5
                % Detect nearby obstacle
                if num_mapping_neighbors < 2
                    if detect_nearest_obstacle(xPos, yPos, unknownObstacleCenters, unknownObstacleRadii, sensing_radius)
                        robot_states(i) = 1; % Switch to mapping state
                        state_change_time(i) = elapsed_time;
                        continue
                    end
                end
            end

            % Obstacle detection
            switch robot_states(i)
                case 1 % Mapping case
                    % Too many robots mapping, switch to Searching
                    if num_mapping_neighbors >= 2
                        robot_states(i) = 0;
                        state_change_time(i) = elapsed_time;
                    else
                        % Random switch back to Searching
                        if rand < p_leave_base && time_since_last_change > 5 % random probability of switching
                            robot_states(i) = 0;
                            state_change_time(i) = elapsed_time;
                            continue;
                        end
            
                        % Circumnavigate the obstacle and map it to the current plot with points
                        % Find the nearest obstacle within sensing radius
                        distances = sqrt((xPos - unknownObstacleCenters(:, 1)).^2 + (yPos - unknownObstacleCenters(:, 2)).^2);
                        [~, closest_idx] = min(distances);
                        [dx, dy] = circumnavigate_obstacle(xPos, yPos, unknownObstacleCenters(closest_idx, :), ...
                            unknownObstacleRadii(closest_idx), sensing_radius);
                        xPos = xPos + dx * dt;
                        yPos = yPos + dy * dt;
                        robot_positions(i, 1) = xPos;
                        robot_positions(i, 2) = yPos;
                        plot(xPos, yPos, 'g.', 'MarkerSize', 8);
                    end
            end
            
            % only random movement in searching state, not mapping
            if robot_states(i) ~= 1
                % boundary repulsion nav function
                [repulsive_dx, repulsive_dy] = compute_obstacle_repulsion(xPos, yPos, nav_dx, nav_dy, x, y);
                if repulsive_dx ~= 0 || repulsive_dy ~= 0
                    dx = repulsive_dx;
                    dy = repulsive_dy;
                else
                    dx = cos(direction_angles(i)) * base_speed;
                    dy = sin(direction_angles(i)) * base_speed;
                end

                % normalize the movement vector
                movement_norm = sqrt(dx^2 + dy^2);
                if movement_norm > 0
                    dx = dx / movement_norm;
                    dy = dy / movement_norm;
                end
    
                % Predict new position
                xPos_new = xPos + dx * dt;
                yPos_new = yPos + dy * dt;

                % Check if the new position is inside any unknown obstacle
                % (NOT PART OF CONTROL - basic logic to ensure robot cannot
                % be inside unknown obstacle
                [inside_obstacle] = check_inside_obstacle(xPos_new, yPos_new, unknownObstacleCenters, unknownObstacleRadii, ...
                    sensing_radius);
                % Update position only if it does not move inside an obstacle
                while inside_obstacle
                    % Try again
                    random_turn = (rand - 0.5) * 2 * (pi/4); % Random turn within [-pi/4, pi/4]
                    direction_angles(i) = direction_angles(i) + random_turn;
                    % Move directly away from the obstacle
                    dx = cos(direction_angles(i)) * base_speed;
                    dy = sin(direction_angles(i)) * base_speed;
                    movement_norm = sqrt(dx^2 + dy^2);
                    if movement_norm > 0
                        dx = dx / movement_norm;
                        dy = dy / movement_norm;
                    end
                    xPos_new = xPos + dx * dt;
                    yPos_new = yPos + dy * dt;
                    % Re-check after adjustment
                    [inside_obstacle] = check_inside_obstacle(xPos_new, yPos_new, unknownObstacleCenters, unknownObstacleRadii, ...
                        sensing_radius);
                end
                robot_positions(i, 1) = xPos_new;
                robot_positions(i, 2) = yPos_new;
            end
        
        % Update robot positions on the plot
        figure(3);
        set(h_robots, 'XData', robot_positions(:, 1), 'YData', robot_positions(:, 2));
        title(['Robot Movement (Time: ', num2str(elapsed_time, '%.1f'), ' s)']);
        pause(0.01); % small delay for visualization
        end
    end
end