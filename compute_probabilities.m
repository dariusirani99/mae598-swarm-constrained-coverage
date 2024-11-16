%% Function for robot leave and join probabilities
function [p_join, p_leave] = compute_probabilities(neighbors, p_encounter_base, p_leave_base)
    % Increase join probability with more neighbors, decrease leave probability
    p_join = p_encounter_base * (1 + neighbors * 0.1);  % Adjust with neighbors
    p_leave = max(p_leave_base / (1 + neighbors), 0.01);  % Minimum leave prob
end
