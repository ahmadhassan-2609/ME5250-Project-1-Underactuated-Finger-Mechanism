% v_linkage_mechanism.m
clear; clc; close all;

% Create figure with UI controls
fig = figure('Position', [100, 100, 1200, 700], 'Name', 'V-Linkage Mechanism');

% Initialize figure data storage
fig.UserData.effector_path = [];
fig.UserData.previous_theta2_long = NaN;

% Slider for input angle (130° to 180°)
slider = uicontrol('Style', 'slider', ...
    'Min', 130, 'Max', 180, 'Value', 130, ...
    'Position', [200, 20, 400, 20], ...
    'SliderStep', [0.02, 0.1], ... % Fine and coarse steps
    'Callback', @update_mechanism);

% UI Labels and controls
uicontrol('Style', 'text', 'Position', [50, 15, 100, 30], ...
    'String', 'Input Angle:', 'FontSize', 10);

% Min and Max labels for slider
uicontrol('Style', 'text', 'Position', [150, 15, 50, 30], ...
    'String', '130°', 'FontSize', 10, 'ForegroundColor', 'b', 'FontWeight', 'bold');
    
uicontrol('Style', 'text', 'Position', [600, 15, 50, 30], ...
    'String', '180°', 'FontSize', 10, 'ForegroundColor', 'r', 'FontWeight', 'bold');

angle_text = uicontrol('Style', 'text', 'Position', [650, 15, 100, 30], ...
    'String', 'Current: 130.0°', 'FontSize', 11, 'FontWeight', 'bold', ...
    'Tag', 'AngleText');

reset_btn = uicontrol('Style', 'pushbutton', ...
    'Position', [750, 15, 100, 30], ...
    'String', 'Reset Path', ...
    'Callback', @reset_path);

% Initial mechanism display
update_mechanism(slider);

function reset_path(src, ~)
    % Get the figure handle
    if nargin > 0 && isvalid(src)
        fig = ancestor(src, 'figure');
    else
        fig = gcf;
    end
    
    % Reset path data
    fig.UserData.effector_path = [];
    fig.UserData.previous_theta2_long = NaN;
    
    % Find and update slider
    slider = findobj(fig, 'Style', 'slider');
    if ~isempty(slider)
        update_mechanism(slider(1));
    end
end

function update_mechanism(slider_obj, ~)
    % Get the figure handle
    fig = ancestor(slider_obj, 'figure');
    
    % Link parameters
    ground_distance = 5.4;    % Distance between ground pivots
    L1_short = 5.4;           % Black V-link short arm
    L1_long = 18;             % Black V-link long arm
    angle1_between = 70;      % Black V-link angle (degrees)

    L2_long = 20.0;           % Blue V-link long arm (to effector)
    L2_short = 6.0;           % Blue V-link short arm
    angle2_between = 60;      % Blue V-link angle (degrees)

    L3 = 15;                  % Gray closing link
    
    % Ground configuration
    ground_Y = -30;           % Ground Y position
    ground_tilt = 20;         % Ground tilt angle in degrees

    % Get input angle from slider
    input_angle_deg = slider_obj.Value;
    angle_text = findobj(fig, 'Tag', 'AngleText');
    if ~isempty(angle_text)
        set(angle_text(1), 'String', sprintf('Current: %.1f°', input_angle_deg));
    end

    % Calculate tilted ground pivot points
    % Right pivot starts at origin but shifted down
    P_ground_right = [0, ground_Y];
    
    % Left pivot is calculated based on ground tilt
    ground_tilt_rad = deg2rad(ground_tilt);
    P_ground_left = P_ground_right + [-ground_distance * cos(ground_tilt_rad), ...
                                       -ground_distance * sin(ground_tilt_rad)];

    % Calculate BLACK V-LINK positions (angle relative to tilted ground)
    % The input angle is now relative to the tilted ground
    theta1 = deg2rad(input_angle_deg) + ground_tilt_rad;
    P1 = P_ground_right + L1_short * [cos(theta1), sin(theta1)];
    theta1_long = theta1 + deg2rad(180 - angle1_between);
    P2 = P1 + L1_long * [cos(theta1_long), sin(theta1_long)];

    % Find valid blue link orientation where P3 is EXACTLY L3 distance from P_ground_left
    % P3 must lie on a circle of radius L3 centered at P_ground_left
    % AND P3 must be L2_short distance from P2
    
    % These two circles must intersect for a valid solution
    circle1_center = P_ground_left;  % Center of L3 circle
    circle1_radius = L3;              % Radius = 15
    circle2_center = P2;              % Center of L2_short circle  
    circle2_radius = L2_short;        % Radius = 6
    
    % Calculate intersection of two circles
    d = norm(circle2_center - circle1_center);
    
    % Check if circles intersect
    if d > (circle1_radius + circle2_radius) || d < abs(circle1_radius - circle2_radius) || d == 0
        % No solution exists - mechanism cannot close at this position
        fprintf('ERROR: Mechanism cannot close at input angle %.1f°!\n', input_angle_deg);
        fprintf('  Distance between P2 and left ground: %.2fmm\n', d);
        fprintf('  Needs to be between %.2f and %.2fmm for closure\n', ...
            abs(L3 - L2_short), L3 + L2_short);
        
        % Draw what we can (without the closing link)
        cla; hold on;
        
        % Draw ground
        ground_extend = 15;
        ground_start = P_ground_right + ground_extend * [cos(ground_tilt_rad + pi), sin(ground_tilt_rad + pi)];
        ground_end = P_ground_left + ground_extend * [cos(ground_tilt_rad), sin(ground_tilt_rad)];
        plot([ground_start(1), ground_end(1)], [ground_start(2), ground_end(2)], 'k-', 'LineWidth', 2);
        
        % Ground pivots
        plot(P_ground_right(1), P_ground_right(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
        plot(P_ground_left(1), P_ground_left(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
        
        % Black V-link
        plot([P_ground_right(1), P1(1)], [P_ground_right(2), P1(2)], 'k-', 'LineWidth', 3);
        plot([P1(1), P2(1)], [P1(2), P2(2)], 'k-', 'LineWidth', 3);
        plot(P1(1), P1(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
        plot(P2(1), P2(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
        
        % Show the problem
        text(P2(1), P2(2)-2, 'Cannot close!', 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
        
        title(sprintf('V-Linkage - CANNOT CLOSE at input angle %.1f°', input_angle_deg), ...
            'FontWeight', 'bold', 'Color', 'r');
        
        axis equal; grid on;
        xlim([-30, 30]); ylim([-60, 20]);
        xlabel('X (mm)', 'FontWeight', 'bold'); 
        ylabel('Y (mm)', 'FontWeight', 'bold');
        
        return;
    end
    
    % Calculate intersection points (only if we get here, circles do intersect)
    a = (circle1_radius^2 - circle2_radius^2 + d^2) / (2*d);
    h_squared = circle1_radius^2 - a^2;
    
    if h_squared < 0
        h_squared = 0; % Handle numerical errors
    end
    h = sqrt(h_squared);
    
    % Point along the line between centers
    P_mid = circle1_center + a * (circle2_center - circle1_center) / d;
    
    % The two possible positions for P3
    offset = h * [-(circle2_center(2) - circle1_center(2)), circle2_center(1) - circle1_center(1)] / d;
    P3_option1 = P_mid + offset;
    P3_option2 = P_mid - offset;
    
    % Choose the appropriate P3 solution
    % Try both intersection points and pick the better one
    
    % Option 1
    P3 = P3_option1;
    v_short = P3 - P2;
    theta2_short_1 = atan2(v_short(2), v_short(1));
    theta2_long_1 = theta2_short_1 - deg2rad(angle2_between);
    
    % Option 2
    v_short2 = P3_option2 - P2;
    theta2_short_2 = atan2(v_short2(2), v_short2(1));
    theta2_long_2 = theta2_short_2 - deg2rad(angle2_between);
    
    % Choose the configuration that points the long arm more downward
    % But accept either solution if it's the only valid one
    angle1_down = abs(rad2deg(theta2_long_1) + 90); % How close to -90° (straight down)
    angle2_down = abs(rad2deg(theta2_long_2) + 90);
    
    if angle1_down <= angle2_down
        P3 = P3_option1;
        best_theta2_short = theta2_short_1;
        best_theta2_long = theta2_long_1;
    else
        P3 = P3_option2;
        best_theta2_short = theta2_short_2;
        best_theta2_long = theta2_long_2;
    end
    
    P_effector = P2 + L2_long * [cos(best_theta2_long), sin(best_theta2_long)];
    
    % Verify the solution maintains rigid link constraint
    actual_L3_distance = norm(P3 - P_ground_left);
    actual_L2_short_distance = norm(P3 - P2);
    
    if abs(actual_L3_distance - L3) > 0.001
        fprintf('Warning: L3 length error: %.4fmm (should be %.1fmm)\n', actual_L3_distance, L3);
    end
    if abs(actual_L2_short_distance - L2_short) > 0.001
        fprintf('Warning: L2_short length error: %.4fmm (should be %.1fmm)\n', actual_L2_short_distance, L2_short);
    end

    % Update path
    fig.UserData.previous_theta2_long = best_theta2_long;
    fig.UserData.effector_path = [fig.UserData.effector_path; P_effector];

    % PLOTTING
    cla; hold on;

    % Draw tilted ground line
    ground_extend = 15;  % How far to extend ground line on each side
    ground_start = P_ground_right + ground_extend * [cos(ground_tilt_rad + pi), sin(ground_tilt_rad + pi)];
    ground_end = P_ground_left + ground_extend * [cos(ground_tilt_rad), sin(ground_tilt_rad)];
    plot([ground_start(1), ground_end(1)], [ground_start(2), ground_end(2)], 'k-', 'LineWidth', 2);
    
    % Add ground angle indicator
    arc_radius = 3;
    arc_angles = linspace(0, ground_tilt_rad, 15);
    arc_x = arc_radius * cos(arc_angles);
    arc_y = ground_Y + arc_radius * sin(arc_angles);
    plot(arc_x, arc_y, 'k--', 'LineWidth', 1);
    text(arc_radius + 1, ground_Y + 1, sprintf('%d°', ground_tilt), ...
        'FontSize', 8, 'Color', 'k');
    
    % Ground pivots
    plot(P_ground_right(1), P_ground_right(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    plot(P_ground_left(1), P_ground_left(2), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
    
    % Black V-link
    plot([P_ground_right(1), P1(1)], [P_ground_right(2), P1(2)], 'k-', 'LineWidth', 3);
    plot([P1(1), P2(1)], [P1(2), P2(2)], 'k-', 'LineWidth', 3);
    plot(P1(1), P1(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'w');

    % Blue V-link
    plot([P2(1), P_effector(1)], [P2(2), P_effector(2)], 'b-', 'LineWidth', 3);
    plot([P2(1), P3(1)], [P2(2), P3(2)], 'b-', 'LineWidth', 3);

    % Gray closing link
    plot([P3(1), P_ground_left(1)], [P3(2), P_ground_left(2)], ...
        'color', [0.5 0.5 0.5], 'LineWidth', 2);

    % Joint markers
    plot(P2(1), P2(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
    plot(P3(1), P3(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'w');
    plot(P_effector(1), P_effector(2), 'r*', 'MarkerSize', 15, 'LineWidth', 2);

    % Draw effector path
    effector_path = fig.UserData.effector_path;
    if size(effector_path, 1) > 1
        path_colors = jet(size(effector_path, 1));
        for i = 2:size(effector_path, 1)
            plot(effector_path(i-1:i,1), effector_path(i-1:i,2), ...
                'Color', path_colors(i,:), 'LineWidth', 2);
        end
        plot(effector_path(1,1), effector_path(1,2), 'go', ...
            'MarkerSize', 8, 'MarkerFaceColor', 'g');
    end

    % Add link length labels
    text(mean([P_ground_right(1), P1(1)]), mean([P_ground_right(2), P1(2)])+0.5, ...
        sprintf('%.1f', L1_short), 'FontSize', 9, 'Color', 'k', 'FontWeight', 'bold');
    text(mean([P1(1), P2(1)]), mean([P1(2), P2(2)])+0.5, ...
        sprintf('%.1f', L1_long), 'FontSize', 9, 'Color', 'k', 'FontWeight', 'bold');
    text(mean([P2(1), P_effector(1)]), mean([P2(2), P_effector(2)])+0.5, ...
        sprintf('%.1f', L2_long), 'FontSize', 9, 'Color', 'b', 'FontWeight', 'bold');
    text(mean([P2(1), P3(1)]), mean([P2(2), P3(2)])-0.5, ...
        sprintf('%.1f', L2_short), 'FontSize', 9, 'Color', 'b', 'FontWeight', 'bold');
    text(mean([P3(1), P_ground_left(1)]), mean([P3(2), P_ground_left(2)])-0.5, ...
        sprintf('%.1f', L3), 'FontSize', 9, 'Color', [0.5 0.5 0.5], 'FontWeight', 'bold');

    % Draw angle arc for blue V-link
    arc_radius = 1.5;
    arc_angles2 = linspace(best_theta2_long, best_theta2_short, 20);
    arc_x2 = P2(1) + arc_radius * cos(arc_angles2);
    arc_y2 = P2(2) + arc_radius * sin(arc_angles2);
    plot(arc_x2, arc_y2, 'b--', 'LineWidth', 1);
    
    mid_angle = (best_theta2_long + best_theta2_short) / 2;
    text(P2(1) + 2*cos(mid_angle), P2(2) + 2*sin(mid_angle), ...
         sprintf('%d°', angle2_between), 'FontSize', 8, 'Color', 'b', 'FontWeight', 'bold');

    % Title with mechanism info
    title(sprintf('V-Linkage (Ground: %.0f° tilt at Y=%.0f) | Input: %.1f° | Effector: [%.1f, %.1f]', ...
        ground_tilt, ground_Y, input_angle_deg, P_effector(1), P_effector(2)), ...
        'FontWeight', 'bold');

    % Set axis properties
    axis equal; 
    grid on;
    xlim([-30, 30]); 
    ylim([-70, 10]);  % Adjusted Y limits for lower ground position
    xlabel('X (mm)', 'FontWeight', 'bold'); 
    ylabel('Y (mm)', 'FontWeight', 'bold');
    
    % Add origin reference
    plot(0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 1);
    text(1, 1, 'Origin', 'FontSize', 8);
    
    % Legend for path
    if size(effector_path, 1) > 1
        legend({'', '', '', '', '', '', '', '', '', '', '', 'Effector Path', 'Start Position'}, ...
            'Location', 'northeast');
    end
end