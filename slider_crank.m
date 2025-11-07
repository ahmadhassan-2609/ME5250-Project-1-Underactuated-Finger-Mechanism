% slider_crank.m
clear; clc; close all;

% Mechanism parameters
r = 12;  % Crank length (mm)
l = 35;  % Connecting rod length (mm)

% ANGLE LIMITS
theta_min = deg2rad(-130);  % Minimum angle (most extended)
theta_max = deg2rad(0);   % Maximum angle (home position)
theta_target = deg2rad(-50);  % Target angle to highlight

% Calculate corresponding slider limits
d_min = r * cos(theta_min) + l * cos(asin(-r * sin(theta_min) / l));
d_max = r * cos(theta_max) + l * cos(asin(-r * sin(theta_max) / l));

% Create figure with slider
fig = figure('Position', [100, 100, 800, 600]);

% Create slider starting at d_max (back position)
slider = uicontrol('Style', 'slider', ...
    'Min', d_min, 'Max', d_max, 'Value', d_max, ...  % Starts at d_max (0 degrees)
    'Position', [150, 20, 500, 20], ...
    'Callback', @(src, event) update_mechanism(src, r, l, theta_min, theta_max, theta_target));

% Add label
uicontrol('Style', 'text', 'Position', [50, 15, 100, 30], ...
    'String', 'Slider Position:', 'FontSize', 10);

% Initial plot
update_mechanism(slider, r, l, theta_min, theta_max, theta_target);

function update_mechanism(slider_obj, r, l, theta_min, theta_max, theta_target)
    % Get slider position
    d = slider_obj.Value;
    
    % Calculate crank angle
    cos_theta = (r^2 + d^2 - l^2) / (2 * r * d);
    
    if abs(cos_theta) > 1
        return;
    end
    
    theta = -acos(cos_theta);  % Negative for downward configuration
    
    % Enforce angle limits
    if theta > theta_max
        theta = theta_max;
    elseif theta < theta_min
        theta = theta_min;
    end
    
    % Check if near -50 degrees
    is_at_target = abs(rad2deg(theta) - (-50)) < 2;  % Within 2 degrees
    
    % Calculate positions
    O = [0, 0];
    A = [r * cos(theta), r * sin(theta)];
    B = [d, 0];
    
    % Clear and plot
    cla; hold on;
    
    % Show angle limits as arc
    theta_range = linspace(theta_min, theta_max, 20);
    arc_x = 15 * cos(theta_range);
    arc_y = 15 * sin(theta_range);
    plot(arc_x, arc_y, 'g--', 'LineWidth', 1);
    
    % Show -50 degree reference line
    target_x = [0, 20 * cos(theta_target)];
    target_y = [0, 20 * sin(theta_target)];
    plot(target_x, target_y, 'm--', 'LineWidth', 2);
    text(target_x(2)+2, target_y(2), '-50°', 'Color', 'm', 'FontWeight', 'bold');
    
    % Plot links with conditional coloring
    if is_at_target
        % Highlight when at -50 degrees
        plot([O(1), A(1)], [O(2), A(2)], 'm-', 'LineWidth', 6);
        plot([A(1), B(1)], [A(2), B(2)], 'm-', 'LineWidth', 6);
    else
        plot([O(1), A(1)], [O(2), A(2)], 'b-', 'LineWidth', 4);
        plot([A(1), B(1)], [A(2), B(2)], 'r-', 'LineWidth', 4);
    end
    
    % Joints
    plot(O(1), O(2), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    plot(A(1), A(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'w');
    plot(B(1), B(2), 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    
    % Ground and slider track
    plot([-10, 50], [0, 0], 'k-', 'LineWidth', 1);
    plot(0, 0, 'k^', 'MarkerSize', 10);
    
    % Display info
    title(sprintf('Slider: %.1f mm | Crank: %.1f° | Range: [%.0f°, %.0f°]', ...
        d, rad2deg(theta), rad2deg(theta_min), rad2deg(theta_max)));
    
    xlabel('X (mm)'); ylabel('Y (mm)');
    axis equal; grid on;
    xlim([-20, 50]); ylim([-20, 10]);
    
    hold off;
end