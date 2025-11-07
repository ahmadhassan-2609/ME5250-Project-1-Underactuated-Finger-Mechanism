% finger_mechanism.m
% Combines slider-crank with switching mechanism (rigid link -> V-linkage at -50°)
clear; clc; close all;

% Define color palette for consistency
colors = struct();
colors.primary = [0.2, 0.4, 0.8];      % Deep blue for primary links
colors.secondary = [0.3, 0.5, 0.9];    % Lighter blue for secondary elements
colors.accent = [0.9, 0.4, 0.2];       % Orange accent for important points
colors.ground = [0.3, 0.3, 0.3];       % Dark gray for ground elements
colors.ghost = [0.7, 0.8, 0.95];       % Light blue-gray for ghost elements
colors.joints = [0.95, 0.95, 0.95];    % Near white for joint fills
colors.effector = [0.1, 0.7, 0.4];     % Green for end effector
colors.reference = [0.6, 0.6, 0.6];    % Medium gray for reference lines
colors.text = [0.4, 0.4, 0.4];         % Text color

% Slider-crank parameters
r = 12;  % Crank length (mm)
l = 35;  % Connecting rod length (mm)

% Angle limits for crank
theta_min = deg2rad(-130);  % Minimum angle (most extended)
theta_max = deg2rad(0);      % Maximum angle (changed to 0 for full range)
theta_switch = deg2rad(-50); % Switching point between mechanisms

% Calculate corresponding slider limits
d_min = r * cos(theta_min) + l * cos(asin(-r * sin(theta_min) / l));
d_max = r * cos(theta_max) + l * cos(asin(-r * sin(theta_max) / l));

% Create figure with modern styling
fig = figure('Position', [100, 100, 1200, 700], ...
    'Color', [0.98, 0.98, 0.98], ...
    'Name', 'Combined Finger Mechanism Visualization');

% Store effector path in figure's UserData instead of global variable
fig.UserData.effector_path = [];

% Create slider with enhanced styling
slider = uicontrol('Style', 'slider', ...
    'Min', d_min, 'Max', d_max, 'Value', d_min, ...
    'Position', [200, 20, 800, 20], ...
    'BackgroundColor', [0.9, 0.9, 0.9], ...
    'Callback', @(src, event) update_combined_mechanism(src, r, l, theta_min, theta_max, theta_switch, colors, fig));

% Add labels with better styling
uicontrol('Style', 'text', 'Position', [50, 15, 150, 30], ...
    'String', 'Actuator Position:', 'FontSize', 11, ...
    'BackgroundColor', [0.98, 0.98, 0.98], ...
    'FontWeight', 'bold');

reset_btn = uicontrol('Style', 'pushbutton', ...
    'Position', [1050, 15, 100, 30], ...
    'String', 'Reset Path', ...
    'BackgroundColor', colors.accent, ...
    'ForegroundColor', 'white', ...
    'FontWeight', 'bold', ...
    'Callback', @(src, event) reset_path(colors, fig));

% Initial plot
update_combined_mechanism(slider, r, l, theta_min, theta_max, theta_switch, colors, fig);

function reset_path(colors, fig_handle)
    % Clear the effector path
    fig_handle.UserData.effector_path = [];
    
    % Find and update the slider
    slider = findobj('Style', 'slider');
    if ~isempty(slider)
        update_combined_mechanism(slider(1), 12, 35, deg2rad(-130), deg2rad(0), deg2rad(-50), colors, fig_handle);
    end
end

function update_combined_mechanism(slider_obj, r, l, theta_min, theta_max, theta_switch, colors, fig_handle)
    % Get slider position
    d = slider_obj.Value;
    
    % Calculate crank angle using inverse kinematics
    cos_theta = (r^2 + d^2 - l^2) / (2 * r * d);
    
    if abs(cos_theta) > 1
        return;
    end
    
    theta_crank = -acos(cos_theta);  % Negative for downward configuration
    
    % Enforce angle limits
    if theta_crank > theta_max
        theta_crank = theta_max;
    elseif theta_crank < theta_min
        theta_crank = theta_min;
    end
    
    % Calculate slider-crank positions
    O = [0, 0];  % Origin/pivot
    A = [r * cos(theta_crank), r * sin(theta_crank)];  % Crank endpoint
    B = [d, 0];  % Slider position
    
    % =================================================================
    % MECHANISM SELECTION BASED ON CRANK ANGLE
    % =================================================================
    
    if theta_crank <= theta_switch
        % =================================================================
        % RIGID LINK ASSEMBLY (from -130° to -50°)
        % =================================================================
        
        % Define rigid link dimensions (same as original)
        L1 = 10;    % Link 1: 10mm at 80 degrees
        L2 = 30.3;  % Link 2: 30.3mm at 200 degrees
        L3 = 5.4;   % Link 3: 5.4mm at 240 degrees
        L5 = 40;    % Extension: 40mm horizontal from P3
        
        % Build the rigid link with ORIGINAL ORIENTATION
        angle1 = deg2rad(80);
        angle2 = deg2rad(200);
        angle3 = deg2rad(240);
        
        % Build initial quadrilateral at origin
        P1_init = [0, 0];  % At origin
        P2_init = P1_init + L1 * [cos(angle1), sin(angle1)];
        P3_init = P2_init + L2 * [cos(angle2), sin(angle2)];
        P4_init = P3_init + L3 * [cos(angle3), sin(angle3)];
        P5_init = P3_init + [-L5, 0];  % 40mm horizontal to the LEFT from P3
        
        % Calculate rotation for the rigid link
        % Map crank angle (-130 to -50) to rigid link rotation (0 to 80 degrees)
        crank_range_rigid = theta_switch - theta_min;
        current_position_rigid = (theta_crank - theta_min) / crank_range_rigid;
        rigid_link_rotation = current_position_rigid * deg2rad(80);
        
        % Rotation matrix for rigid link
        R = [cos(rigid_link_rotation), -sin(rigid_link_rotation);
             sin(rigid_link_rotation),  cos(rigid_link_rotation)];
        
        % Rotate the rigid link assembly
        P1 = P1_init;  % P1 stays at origin
        P2 = (R * P2_init')';
        P3 = (R * P3_init')';
        P4 = (R * P4_init')';
        P5 = (R * P5_init')';  % End effector
        P_effector = P5;
        
        mechanism_type = 'Rigid Link';
        mechanism_info = sprintf('Rotation: %.1f°', rad2deg(rigid_link_rotation));
        
    else
        % =================================================================
        % V-LINKAGE MECHANISM (from -50° to 0°)
        % Connects at P4 position from rigid link at -50°
        % =================================================================
        
        % First, calculate where P4 is at the switching point (-50°)
        L1 = 10;    
        L2 = 30.3;  
        L3_rigid = 5.4;   
        angle1 = deg2rad(80);
        angle2 = deg2rad(200);
        angle3 = deg2rad(240);
        
        P1_init = [0, 0];
        P2_init = P1_init + L1 * [cos(angle1), sin(angle1)];
        P3_init = P2_init + L2 * [cos(angle2), sin(angle2)];
        P4_init = P3_init + L3_rigid * [cos(angle3), sin(angle3)];
        
        % At -50°, rigid link is at 80° rotation
        R_switch = [cos(deg2rad(80)), -sin(deg2rad(80));
                    sin(deg2rad(80)),  cos(deg2rad(80))];
        P4_at_switch = (R_switch * P4_init')';  % Position of P4 at switching point
        
        % V-linkage parameters
        ground_distance = 5.4;    % Distance between ground pivots (matches L3!)
        L1_short = 5.4;           % Black V-link short arm
        L1_long = 18;             % Black V-link long arm
        angle1_between = 70;      % Black V-link angle (degrees)
        
        L2_long = 20.0;           % Blue V-link long arm (to effector)
        L2_short = 6.0;           % Blue V-link short arm
        angle2_between = 60;      % Blue V-link angle (degrees)
        
        L3 = 15;                  % Gray closing link
        
        % Ground configuration
        ground_tilt = 20;         % Ground tilt angle in degrees (CHANGED FROM 25 TO 20)
        
        % Map crank angle (-50 to 0) to V-linkage input with alignment correction
        crank_range_v = theta_max - theta_switch;
        current_position_v = (theta_crank - theta_switch) / crank_range_v;
        % Add 5 degree clockwise offset to align with rigid link at switch point
        alignment_offset = 5;  % Small clockwise rotation to match dotted line
        input_angle_deg = (130 + alignment_offset) + current_position_v * 50;
        
        % Calculate tilted ground pivot points
        % Right pivot connects to P4_at_switch position
        P_ground_right = P4_at_switch;  % V-linkage connects at P4 position!
        
        ground_tilt_rad = deg2rad(ground_tilt);
        P_ground_left = P_ground_right + [-ground_distance * cos(ground_tilt_rad), ...
                                           -ground_distance * sin(ground_tilt_rad)];
        
        % Calculate BLACK V-LINK positions
        theta1 = deg2rad(input_angle_deg) + ground_tilt_rad;
        P1 = P_ground_right + L1_short * [cos(theta1), sin(theta1)];
        theta1_long = theta1 + deg2rad(180 - angle1_between);
        P2 = P1 + L1_long * [cos(theta1_long), sin(theta1_long)];
        
        % Also keep drawing the rigid link in ghost form to show connection
        P1_rigid = [0, 0];
        P2_rigid = (R_switch * P2_init')';
        P3_rigid = (R_switch * P3_init')';
        P4_rigid = P4_at_switch;
        
        % Find BLUE V-LINK orientation
        circle1_center = P_ground_left;  
        circle1_radius = L3;              
        circle2_center = P2;              
        circle2_radius = L2_short;        
        
        d_circles = norm(circle2_center - circle1_center);
        
        if d_circles > (circle1_radius + circle2_radius) || d_circles < abs(circle1_radius - circle2_radius) || d_circles == 0
            % Fallback if V-linkage can't close
            P3 = P3_rigid;
            P4 = P4_rigid;
            P_effector = P3_rigid + [-40, 0];
            mechanism_type = 'V-Link (Error)';
            mechanism_info = 'Cannot close';
        else
            % Calculate intersection points
            a = (circle1_radius^2 - circle2_radius^2 + d_circles^2) / (2*d_circles);
            h = sqrt(circle1_radius^2 - a^2);
            
            P_mid = circle1_center + a * (circle2_center - circle1_center) / d_circles;
            offset = h * [-(circle2_center(2) - circle1_center(2)), circle2_center(1) - circle1_center(1)] / d_circles;
            
            P3_option1 = P_mid + offset;
            P3_option2 = P_mid - offset;
            
            % Choose the better P3 solution
            v_short1 = P3_option1 - P2;
            theta2_short_1 = atan2(v_short1(2), v_short1(1));
            theta2_long_1 = theta2_short_1 - deg2rad(angle2_between);
            
            v_short2 = P3_option2 - P2;
            theta2_short_2 = atan2(v_short2(2), v_short2(1));
            theta2_long_2 = theta2_short_2 - deg2rad(angle2_between);
            
            angle1_down = abs(rad2deg(theta2_long_1) + 90);
            angle2_down = abs(rad2deg(theta2_long_2) + 90);
            
            if angle1_down <= angle2_down
                P3 = P3_option1;
                best_theta2_long = theta2_long_1;
            else
                P3 = P3_option2;
                best_theta2_long = theta2_long_2;
            end
            
            P_effector = P2 + L2_long * [cos(best_theta2_long), sin(best_theta2_long)];
            P4 = P_ground_left;  % For V-linkage, P4 is the left ground pivot
            
            mechanism_type = 'V-Linkage';
            mechanism_info = sprintf('Input: %.1f°', input_angle_deg);
        end
    end
    
    % Update path (retrieve and update from figure's UserData)
    effector_path = fig_handle.UserData.effector_path;
    effector_path = [effector_path; P_effector];
    fig_handle.UserData.effector_path = effector_path;
    
    % =================================================================
    % PLOTTING WITH ENHANCED VISUALS
    % =================================================================
    
    cla; hold on;
    
    % Set background color
    set(gca, 'Color', [0.98, 0.98, 0.98]);
    
    % Draw angle reference arcs for crank (subtle)
    theta_range = linspace(theta_min, theta_max, 30);
    arc_x_crank = 15 * cos(theta_range);
    arc_y_crank = 15 * sin(theta_range);
    plot(arc_x_crank, arc_y_crank, '--', 'LineWidth', 1, 'Color', colors.reference);
    
    % Reference lines with enhanced styling
    % Switch line
    plot([0, 20 * cos(deg2rad(-50))], [0, 20 * sin(deg2rad(-50))], ...
        '--', 'LineWidth', 2, 'Color', colors.accent);
    text(24 * cos(deg2rad(-50)), 24 * sin(deg2rad(-50)), 'SWITCH', ...
        'Color', colors.accent, 'FontSize', 10, 'FontWeight', 'bold');
    
    % Angle limits
    plot([0, 20 * cos(deg2rad(-130))], [0, 20 * sin(deg2rad(-130))], ...
        '--', 'LineWidth', 1, 'Color', colors.reference);
    text(24 * cos(deg2rad(-130)), 24 * sin(deg2rad(-130)), '-130°', ...
        'Color', colors.text, 'FontSize', 9);
    
    plot([0, 20 * cos(deg2rad(0))], [0, 20 * sin(deg2rad(0))], ...
        '--', 'LineWidth', 1, 'Color', colors.reference);
    text(24, 0, '0°', 'Color', colors.text, 'FontSize', 9);
    
    % ===== SLIDER-CRANK MECHANISM (always visible) =====
    plot([O(1), A(1)], [O(2), A(2)], '-', 'LineWidth', 4, 'Color', colors.primary);
    plot([A(1), B(1)], [A(2), B(2)], '-', 'LineWidth', 3.5, 'Color', colors.secondary);
    
    % Slider-crank joints
    plot(A(1), A(2), 'o', 'MarkerSize', 10, ...
        'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.joints, 'LineWidth', 2);
    plot(B(1), B(2), 's', 'MarkerSize', 11, ...
        'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.effector, 'LineWidth', 2);
    
    % Ground and slider track
    plot([-40, 60], [0, 0], '-', 'LineWidth', 3, 'Color', colors.ground);
    plot(0, 0, '^', 'MarkerSize', 14, ...
        'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.ground);
    
    % ===== MECHANISM-SPECIFIC DRAWING =====
    if theta_crank <= theta_switch
        % Draw RIGID LINK with consistent styling
        
        % Ghost outline for reference
        quad_x_init = [P1_init(:,1); P2_init(:,1); P3_init(:,1); P4_init(:,1); P1_init(:,1)];
        quad_y_init = [P1_init(:,2); P2_init(:,2); P3_init(:,2); P4_init(:,2); P1_init(:,2)];
        plot(quad_x_init, quad_y_init, '--', 'LineWidth', 1, 'Color', colors.ghost);
        
        % Current position with consistent color
        quad_x = [P1(1), P2(1), P3(1), P4(1), P1(1)];
        quad_y = [P1(2), P2(2), P3(2), P4(2), P1(2)];
        plot(quad_x, quad_y, '-', 'Color', colors.primary, 'LineWidth', 3);
        
        % Extension links
        plot([P3(1), P5(1)], [P3(2), P5(2)], '-', 'LineWidth', 3, 'Color', colors.secondary);
        plot([P5(1), P4(1)], [P5(2), P4(2)], '-', 'LineWidth', 3, 'Color', colors.secondary);
        
        % Joints with consistent styling
        plot(P2(1), P2(2), 'o', 'MarkerSize', 8, ...
            'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.joints, 'LineWidth', 1.5);
        plot(P3(1), P3(2), 'o', 'MarkerSize', 8, ...
            'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.joints, 'LineWidth', 1.5);
        plot(P4(1), P4(2), 'o', 'MarkerSize', 8, ...
            'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.joints, 'LineWidth', 1.5);
        
        % Subtle dimension labels
        text(mean([P1(1), P2(1)]), mean([P1(2), P2(2)])+1, '10', ...
            'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
        text(mean([P2(1), P3(1)]), mean([P2(2), P3(2)])+1, '30.3', ...
            'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
        text(mean([P3(1), P4(1)]), mean([P3(2), P4(2)])-1, '5.4', ...
            'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
        text(mean([P3(1), P5(1)]), mean([P3(2), P5(2)])+1, '40', ...
            'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
        
    else
        % Draw V-LINKAGE with consistent styling
        
        % Ghost of rigid link at switch position
        quad_x_ghost = [P1_rigid(1), P2_rigid(1), P3_rigid(1), P4_rigid(1), P1_rigid(1)];
        quad_y_ghost = [P1_rigid(2), P2_rigid(2), P3_rigid(2), P4_rigid(2), P1_rigid(2)];
        plot(quad_x_ghost, quad_y_ghost, '--', 'LineWidth', 1, 'Color', colors.ghost);
        
        % Draw ground line
        ground_extend = 8;
        ground_tilt_rad = deg2rad(20);  % CHANGED FROM 25 TO 20
        ground_start = P_ground_right + ground_extend * [cos(ground_tilt_rad + pi), sin(ground_tilt_rad + pi)];
        ground_end = P_ground_left + ground_extend * [cos(ground_tilt_rad), sin(ground_tilt_rad)];
        plot([ground_start(1), ground_end(1)], [ground_start(2), ground_end(2)], ...
            '-', 'LineWidth', 3, 'Color', colors.ground);
        
        % Ground pivots
        plot(P_ground_right(1), P_ground_right(2), 'o', 'MarkerSize', 11, ...
            'MarkerEdgeColor', colors.accent, 'MarkerFaceColor', colors.accent, 'LineWidth', 2);
        text(P_ground_right(1)+3, P_ground_right(2), 'P4', ...
            'Color', colors.accent, 'FontSize', 9, 'FontWeight', 'bold');
        plot(P_ground_left(1), P_ground_left(2), 'o', 'MarkerSize', 11, ...
            'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.ground);
        
        % V-links with consistent colors
        % First V-link
        plot([P_ground_right(1), P1(1)], [P_ground_right(2), P1(2)], ...
            '-', 'LineWidth', 3, 'Color', colors.primary);
        plot([P1(1), P2(1)], [P1(2), P2(2)], '-', 'LineWidth', 3, 'Color', colors.primary);
        
        % Second V-link  
        plot([P2(1), P_effector(1)], [P2(2), P_effector(2)], ...
            '-', 'LineWidth', 3, 'Color', colors.secondary);
        plot([P2(1), P3(1)], [P2(2), P3(2)], '-', 'LineWidth', 3, 'Color', colors.secondary);
        
        % Closing link
        plot([P3(1), P_ground_left(1)], [P3(2), P_ground_left(2)], ...
            '-', 'Color', colors.primary, 'LineWidth', 2.5);
        
        % Joints
        plot(P1(1), P1(2), 'o', 'MarkerSize', 8, ...
            'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.joints, 'LineWidth', 1.5);
        plot(P2(1), P2(2), 'o', 'MarkerSize', 8, ...
            'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.joints, 'LineWidth', 1.5);
        plot(P3(1), P3(2), 'o', 'MarkerSize', 8, ...
            'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.joints, 'LineWidth', 1.5);
        
        % Subtle dimension labels
        text(mean([P_ground_right(1), P1(1)]), mean([P_ground_right(2), P1(2)])+1, ...
            '5.4', 'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
        text(mean([P1(1), P2(1)]), mean([P1(2), P2(2)])+1, ...
            '18', 'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
        text(mean([P2(1), P_effector(1)])-1, mean([P2(2), P_effector(2)])+1, ...
            '20', 'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
        text(mean([P2(1), P3(1)])+1, mean([P2(2), P3(2)])-1, ...
            '6', 'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
        text(mean([P3(1), P_ground_left(1)]), mean([P3(2), P_ground_left(2)])-1, ...
            '15', 'FontSize', 8, 'Color', colors.text, 'HorizontalAlignment', 'center');
    end
    
    % Main pivot (always on top)
    plot(O(1), O(2), 's', 'MarkerSize', 14, ...
        'MarkerEdgeColor', colors.ground, 'MarkerFaceColor', colors.ground);
    
    % End effector with emphasis
    plot(P_effector(1), P_effector(2), 'p', 'MarkerSize', 16, ...
        'MarkerEdgeColor', colors.effector, 'MarkerFaceColor', colors.effector, 'LineWidth', 2);
    
    % Draw effector path with gradient
    if size(effector_path, 1) > 1
        % Create smooth gradient from blue to green
        n_points = size(effector_path, 1);
        for i = 2:n_points
            t = (i-1) / (n_points-1);  % Normalized position
            path_color = (1-t) * colors.secondary + t * colors.effector;
            plot(effector_path(i-1:i,1), effector_path(i-1:i,2), ...
                'Color', path_color, 'LineWidth', 2.5);
        end
        % Mark start point
        plot(effector_path(1,1), effector_path(1,2), 'o', ...
            'MarkerSize', 8, 'MarkerEdgeColor', colors.secondary, ...
            'MarkerFaceColor', colors.secondary);
    end
    
    % Title
    title_str = sprintf('Crank: %.1f° | %s (%s) | End Effector: [%.1f, %.1f]', ...
        rad2deg(theta_crank), mechanism_type, mechanism_info, P_effector(1), P_effector(2));
    title(title_str, 'FontWeight', 'bold', 'FontSize', 12, 'Color', colors.text);
    
    % Axis labels with styling
    xlabel('X (mm)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', colors.text);
    ylabel('Y (mm)', 'FontSize', 11, 'FontWeight', 'bold', 'Color', colors.text);
    
    % Grid and axis settings
    grid on;
    set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.3, 'GridColor', colors.reference);
    axis equal;
    xlim([-80, 60]);
    ylim([-80, 30]);
    
    % Mechanism phase indicator
    if theta_crank <= theta_switch
        text(30, -50, 'Phase 1: Rigid Link', ...
            'FontSize', 11, 'FontWeight', 'bold', 'Color', colors.primary);
        text(30, -54, 'Range: [-130°, -50°]', ...
            'FontSize', 9, 'Color', colors.text);
    else
        text(30, -50, 'Phase 2: V-Linkage', ...
            'FontSize', 11, 'FontWeight', 'bold', 'Color', colors.secondary);
        text(30, -54, 'Range: [-50°, 0°]', ...
            'FontSize', 9, 'Color', colors.text);
        text(30, -58, 'Connected at P4', ...
            'FontSize', 8, 'Color', colors.accent);
    end
    
    % Add box with rounded appearance
    box on;
    set(gca, 'LineWidth', 1, 'XColor', colors.reference, 'YColor', colors.reference);
    
    hold off;
end