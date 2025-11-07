% rigid_quad_linkage.m
clear; clc; close all;

% Create figure with slider
fig = figure('Position', [100, 100, 900, 600]);

slider = uicontrol('Style', 'slider', ...
    'Min', 0, 'Max', 80, 'Value', 0, ...  % 0 to 80 degree range
    'Position', [150, 20, 600, 20], ...
    'Callback', @(src, event) update_quad(src));

uicontrol('Style', 'text', 'Position', [50, 15, 100, 30], ...
    'String', 'Rotation:', 'FontSize', 10);

update_quad(slider);

function update_quad(slider_obj)
    % Link lengths
    L1 = 10;    % at 80 degrees
    L2 = 30.3;  % at 200 degrees
    L3 = 5.4;   % at 240 degrees
    L5 = 40;    % NEW: horizontal extension from P3
    
    % Get rotation angle (0 to 80 degrees, anticlockwise)
    rotation_deg = slider_obj.Value;
    % Anticlockwise rotation (positive angle)
    theta_rotation = deg2rad(rotation_deg);
    
    % Build initial quadrilateral
    P1 = [0, 0];  % Pivot (ground point)
    
    % Link 1: 10mm at 80 degrees
    angle1 = deg2rad(80);
    P2 = P1 + L1 * [cos(angle1), sin(angle1)];
    
    % Link 2: 30.3mm at 200 degrees from P2
    angle2 = deg2rad(200);
    P3 = P2 + L2 * [cos(angle2), sin(angle2)];
    
    % Link 3: 5.4mm at 240 degrees from P3
    angle3 = deg2rad(240);
    P4 = P3 + L3 * [cos(angle3), sin(angle3)];
    
    % NEW: Link 5 - 40mm horizontal to the LEFT from P3
    P5 = P3 + [-L5, 0];  % Horizontal to the left means subtracting from x-coordinate
    
    vertices_initial = [P1; P2; P3; P4];
    extension_initial = P5;  % New end effector position
    green_cross_initial = P5;  % Green cross now at P5
    
    % Rotation matrix for ANTICLOCKWISE rotation
    R = [cos(theta_rotation), -sin(theta_rotation);
         sin(theta_rotation),  cos(theta_rotation)];
    
    % Rotate everything around pivot
    vertices_rotated = zeros(size(vertices_initial));
    vertices_rotated(1,:) = vertices_initial(1,:);  % Pivot stays fixed
    
    % Rotate other vertices around pivot
    for i = 2:4
        relative_pos = vertices_initial(i,:) - vertices_initial(1,:);
        rotated_pos = (R * relative_pos')';
        vertices_rotated(i,:) = rotated_pos + vertices_initial(1,:);
    end
    
    % Rotate P5 (extension point)
    extension_relative = extension_initial - vertices_initial(1,:);
    extension_rotated = (R * extension_relative')' + vertices_initial(1,:);
    
    % Rotate green cross (now at P5)
    green_cross_relative = green_cross_initial - vertices_initial(1,:);
    green_cross_rotated = (R * green_cross_relative')' + vertices_initial(1,:);
    
    % Plot
    cla; hold on;
    
    % Show initial position (ghost) - quadrilateral
    quad_x_init = [vertices_initial(:,1); vertices_initial(1,1)];
    quad_y_init = [vertices_initial(:,2); vertices_initial(1,2)];
    plot(quad_x_init, quad_y_init, 'b--', 'LineWidth', 1, 'Color', [0.7 0.7 1]);
    
    % Show initial extension (ghost)
    plot([vertices_initial(3,1), extension_initial(1)], ...
         [vertices_initial(3,2), extension_initial(2)], ...
         'm--', 'LineWidth', 1, 'Color', [1 0.7 1]);
    plot([extension_initial(1), vertices_initial(4,1)], ...
         [extension_initial(2), vertices_initial(4,2)], ...
         'm--', 'LineWidth', 1, 'Color', [1 0.7 1]);
    
    % Draw rotated quadrilateral
    quad_x = [vertices_rotated(:,1); vertices_rotated(1,1)];
    quad_y = [vertices_rotated(:,2); vertices_rotated(1,2)];
    plot(quad_x, quad_y, 'b-', 'LineWidth', 3);
    
    % Draw extension links
    % Link from P3 to P5 (horizontal extension)
    plot([vertices_rotated(3,1), extension_rotated(1)], ...
         [vertices_rotated(3,2), extension_rotated(2)], ...
         'm-', 'LineWidth', 3);
    
    % Link from P5 to P4 (connecting back)
    plot([extension_rotated(1), vertices_rotated(4,1)], ...
         [extension_rotated(2), vertices_rotated(4,2)], ...
         'm-', 'LineWidth', 3);
    
    % Mark vertices
    plot(vertices_rotated(2:4,1), vertices_rotated(2:4,2), 'ko', ...
        'MarkerSize', 8, 'MarkerFaceColor', 'w');
    
    % Mark P5 (extension point)
    plot(extension_rotated(1), extension_rotated(2), 'mo', ...
        'MarkerSize', 10, 'MarkerFaceColor', 'm');
    
    % Pivot (square marker)
    plot(vertices_rotated(1,1), vertices_rotated(1,2), 'ks', ...
        'MarkerSize', 12, 'MarkerFaceColor', 'k');
    
    % Green cross (now at P5)
    plot(green_cross_rotated(1), green_cross_rotated(2), 'g*', ...
        'MarkerSize', 20, 'LineWidth', 3);
    
    % Initial green cross position (ghost)
    plot(green_cross_initial(1), green_cross_initial(2), 'g*', ...
        'MarkerSize', 10, 'LineWidth', 1, 'Color', [0.7 1 0.7]);
    
    % Add rotation arrow to show direction
    if rotation_deg > 0
        arc_angles = linspace(0, theta_rotation, 20);
        arc_radius = 8;
        arc_x = arc_radius * cos(arc_angles);
        arc_y = arc_radius * sin(arc_angles);
        plot(arc_x, arc_y, 'k-', 'LineWidth', 2);
        % Arrow head
        plot(arc_x(end), arc_y(end), 'k>', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    end
    
    % Labels for quadrilateral sides
    side_labels = {'10', '30.3', '5.4', '30.1'};
    for i = 1:4
        j = mod(i, 4) + 1;
        mid_x = (vertices_rotated(i,1) + vertices_rotated(j,1))/2;
        mid_y = (vertices_rotated(i,2) + vertices_rotated(j,2))/2;
        text(mid_x, mid_y, side_labels{i}, ...
            'FontSize', 10, 'Color', 'red', 'FontWeight', 'bold');
    end
    
    % Label for horizontal extension (40mm)
    mid_ext_x = (vertices_rotated(3,1) + extension_rotated(1))/2;
    mid_ext_y = (vertices_rotated(3,2) + extension_rotated(2))/2;
    text(mid_ext_x, mid_ext_y, '40', ...
        'FontSize', 10, 'Color', 'magenta', 'FontWeight', 'bold');
    
    % Calculate and label the connecting link length (P5 to P4)
    connecting_length = norm(extension_initial - vertices_initial(4,:));
    mid_conn_x = (extension_rotated(1) + vertices_rotated(4,1))/2;
    mid_conn_y = (extension_rotated(2) + vertices_rotated(4,2))/2;
    text(mid_conn_x, mid_conn_y, sprintf('%.1f', connecting_length), ...
        'FontSize', 10, 'Color', 'magenta', 'FontWeight', 'bold');
    
    title(sprintf('Anticlockwise Rotation: %.1f° | End Effector (Green): [%.1f, %.1f]', ...
        rotation_deg, green_cross_rotated(1), green_cross_rotated(2)));
    
    axis equal; grid on;
    xlim([-75, 15]); ylim([-80, 15]);
    grid on;
end