%% Visualize Repulsive Field Function
function visualize_repulsive_field(X, Y, nav_function)
    % Function to visualize the repulsive field as a surface plot
    figure;
    surf(X, Y, nav_function, 'EdgeColor', 'none');
    colorbar;
    xlabel('X');
    ylabel('Y');
    zlabel('Navigation Function');
    title('Surface Plot of the Shape Navigation Function');
    axis([-200 200 -200 200 0 1]);
    view(3);
end
