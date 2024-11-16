%% Visualize Repulsive Field Function
function visualize_repulsive_field(X, Y, repulsive_field)
    % Function to visualize the repulsive field as a surface plot
    figure;
    surf(X, Y, repulsive_field, 'EdgeColor', 'none'); % Surface plot of the repulsive field
    colorbar;
    xlabel('X');
    ylabel('Y');
    zlabel('Repulsive Field Intensity');
    title('Visualization of the Obstacle Repulsive Field');
    view(3); % Set to 3D view for better visualization of field intensity
    datacursormode on;
end
