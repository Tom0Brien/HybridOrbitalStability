close all;
clear;
clc;

%% Parameters
syms q1 q2 p1 p2 
sys.q = [q1;q2];
sys.p = [p1;p2];

% Dynamics
sys.m = 1;
sys.M = sys.m*eye(2);
sys.g = 9.81;
sys.V = @(q) sys.m*sys.g*q(2);
sys.H = @(q,p) p.'*M*p + V(q);
sys.J = [zeros(2), eye(2); -eye(2), zeros(2)];

sys.dHdq = [0; sys.m*sys.g];
sys.dHdp = @(p) [p(1)/sys.m; p(2)/sys.m];
sys.G = eye(2);

% Hybrid conditions
sys.a = -1;
sys.b = 1;
sys.phase = 1; % 1 = go right, -1 go left
sys.P = [-1,0;0,1];

% Controller gains
sys.K_p = 2*eye(2);
sys.K_d = 1*eye(2);

%% Initial conditions
x0 = [-0.1; 0.25; 0.5; 0];  % Initial state ([q1; q2; p1; p2])

%% Integrate system with ode45 and event detection
impact_event = @(t,x) collision_events(t,x,sys);
options = odeset('Events', impact_event, 'RelTol', 1e-6, 'AbsTol', 1e-6);
T = 10;  % Simulation time
tspan = [0, T];
t_out = [];
x_out = [];
phase_out = [];
event_indices = [];
while tspan(1) < T
    [t, x] = ode45(@(t, x) system_dynamics(t, x, sys), tspan, x0, options);
    t_out = [t_out; t];
    x_out = [x_out; x];
    phase_out = [phase_out; repmat(sys.phase, length(t), 1)];
    
    if ~isempty(t) && t(end) == tspan(2)
        break; % Simulation finished
    end

    % Record the event index
    event_indices = [event_indices; length(t_out)];

    % Apply impact mapping
    x_minus= x(end, :)';
    x0 = impact_mapping(x_minus,sys);
    sys.phase = sys.phase * -1; % Flip directon of desired velocity
    tspan(1) = t(end);
end

% Define the color palette
colors = ['#440087'; '#a5d0ea'; '#D4FFFD'; '#7DEEF1'];

% Function to convert hex color code to RGB
hexToRGB = @(hex) sscanf(hex(2:end), '%2x') / 255;

%% Plotting
qd_out = [ x_out(:,1), 0.5 .* phase_out .* sin(pi*(x_out(:,1)-sys.a)/(sys.b-sys.a)) ];
pd_out = zeros(length(t_out),2);
for i = 1:length(t_out)
    q1 = x_out(i,1);
    phase_val = phase_out(i);  % current phase value (either 1 or -1)
    % Compute the reference configuration (for clarity, though q1 is used directly)
    qd = [q1; 0.5 * phase_val * sin(pi*(q1 - sys.a)/(sys.b - sys.a))];
    % Compute the derivative of q_d with respect to q1:
    %   derivative = [1; 0.5*phase_val*cos(pi*(q1-sys.a)/(sys.b-sys.a))*(pi/(sys.b-sys.a))]
    % Then, as in your controller, the desired velocity is defined as
    %   v_star = phase_val * (derivative) = [phase_val; 0.5*cos(pi*(q1-sys.a)/(sys.b-sys.a))*(pi/(sys.b-sys.a))]
    v_star = [phase_val; 0.5  * cos(pi*(q1 - sys.a)/(sys.b - sys.a)) * (pi/(sys.b - sys.a))];
    % Since the mass matrix M is the identity, the desired momentum is simply v_star.
    pd_out(i,:) = v_star';
end

% Plot closed loop energy
H_tilde_out = zeros(size(t_out));
q_tilde_out = zeros(length(t_out),2);
p_tilde_out = zeros(length(t_out),2);
qd_out = zeros(length(t_out),2);
for i = 1:(length(t_out))
    sys.phase = phase_out(i);
    sys.qd = @(q) [q(1); 0.5 * sys.phase * sin(pi * (q(1) - sys.a) / (sys.b - sys.a))];
    sys.qtilde = @(q) q - sys.qd(q);
    qd_out(i, :) = sys.qd(x_out(i, 1:2)');
    sys.qdotstar = @(q) sys.phase*subs(jacobian(sys.qd(sys.q),sys.q(1)),sys.q(1),q(1));
    sys.dqtildedq = @(q) subs(jacobian(sys.qtilde(sys.q),sys.q),sys.q, q);
    sys.pd = @(q) sys.M*(sys.qdotstar(q) - sys.K_p*sys.dqtildedq(q).'*sys.qtilde(q));
    sys.dpddq = @(q) subs(jacobian(sys.pd(sys.q),sys.q),sys.q,q);
    sys.ptilde = @(q,p) p - sys.pd(q);
    sys.H_tilde = @(q,p) 0.5*sys.qtilde(q).'*sys.K_p*sys.qtilde(q) + 0.5*sys.ptilde(q,p).'*sys.ptilde(q,p);q = x_out(i, 1:2)';
    p = x_out(i, 3:4)';
    H_tilde_out(i) = sys.H_tilde(q, p);
    q_tilde_out(i,:) = sys.qtilde(q);
    p_tilde_out(i,:) = sys.ptilde(q,p);
end





%% Plotting
figure;

% Define the new color scheme for alternating segments
colors = ['#a5d0ea'; '#c2e5ce'];
hexToRGB = @(hex) sscanf(hex(2:end), '%2x') / 255;
labelColor = '#545454';

% Prepare segment indices for alternating colors
segment_indices = [1; event_indices; length(t_out)];
num_segments = length(segment_indices) - 1;

% Plot q1 vs time
subplot(4,1,1);
hold on;
for i = 1:num_segments
    idx_start = segment_indices(i);
    idx_end = segment_indices(i+1);
    color_idx = mod(i-1, 2) + 1; % Alternate between 1 and 2
    plot(t_out(idx_start:idx_end), x_out(idx_start:idx_end,1), 'LineWidth', 5, 'Color', hexToRGB(colors(color_idx,:)));
end
plot(t_out, qd_out(:,1), '--', 'LineWidth', 4, 'Color', labelColor); % reference q1
xlabel('$t\, (s)$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
ylabel('$q_1$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
ax = gca;
ax.XColor = labelColor;
ax.YColor = labelColor;

% Plot q2 vs time
subplot(4,1,2);
hold on;
for i = 1:num_segments
    idx_start = segment_indices(i);
    idx_end = segment_indices(i+1);
    color_idx = mod(i-1, 2) + 1; % Alternate between 1 and 2
    plot(t_out(idx_start:idx_end), x_out(idx_start:idx_end,2), 'LineWidth', 5, 'Color', hexToRGB(colors(color_idx,:)));
end
plot(t_out, qd_out(:,2), '--', 'LineWidth', 4, 'Color', labelColor); % reference q2
xlabel('$t\, (s)$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
ylabel('$q_2$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
ax = gca;
ax.XColor = labelColor;
ax.YColor = labelColor;

% Plot p1 vs time (momentum in q1-direction)
subplot(4,1,3);
hold on;
for i = 1:num_segments
    idx_start = segment_indices(i);
    idx_end = segment_indices(i+1);
    color_idx = mod(i-1, 2) + 1; % Alternate between 1 and 2
    plot(t_out(idx_start:idx_end), x_out(idx_start:idx_end,3), 'LineWidth', 5, 'Color', hexToRGB(colors(color_idx,:)));
end
plot(t_out, pd_out(:,1), '--', 'LineWidth', 4, 'Color', labelColor); % reference p1
xlabel('$t\, (s)$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
ylabel('$p_1$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
ax = gca;
ax.XColor = labelColor;
ax.YColor = labelColor;

subplot(4,1,4);
hold on;
leftLegendPlotted = false;
rightLegendPlotted = false;
for i = 1:num_segments
    idx_start = segment_indices(i);
    idx_end = segment_indices(i+1);
    color_idx = mod(i-1, 2) + 1; % Alternate between 1 and 2
    if color_idx == 1 && ~leftLegendPlotted
         plot(t_out(idx_start:idx_end), x_out(idx_start:idx_end,4), ...
              'LineWidth', 5, 'Color', hexToRGB(colors(color_idx,:)), ...
              'DisplayName', 'Actual $v_l$');
         leftLegendPlotted = true;
    elseif color_idx == 2 && ~rightLegendPlotted
         plot(t_out(idx_start:idx_end), x_out(idx_start:idx_end,4), ...
              'LineWidth', 5, 'Color', hexToRGB(colors(color_idx,:)), ...
              'DisplayName', 'Actual $v_r$');
         rightLegendPlotted = true;
    else
         plot(t_out(idx_start:idx_end), x_out(idx_start:idx_end,4), ...
              'LineWidth', 5, 'Color', hexToRGB(colors(color_idx,:)), ...
              'HandleVisibility','off');
    end
end
plot(t_out, pd_out(:,2), '--', 'LineWidth', 4, ...
     'Color', labelColor, 'DisplayName', 'Reference'); % reference q1
xlabel('$t\, (s)$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
ylabel('$q_1$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
ax = gca;
ax.XColor = labelColor;
ax.YColor = labelColor;
legend('show', 'Interpreter', 'latex', 'FontSize', 16, 'TextColor', labelColor);

% Format the Figure for Publication
set(gcf, 'Color', 'w');  
set(gcf, 'Units', 'inches', 'Position', [1 1 6 6]);  % [left bottom width height]

% Export the figure
exportgraphics(gcf, 'closed_loop_simulation.png', 'Resolution', 300);
exportgraphics(gcf, 'closed_loop_simulation.eps', 'ContentType', 'vector');

% Plot closed loop energy on a logarithmic scale with alternating segment colors
figure;
hold on;
left_plotted = false;
right_plotted = false;
for i = 1:num_segments
    idx_start = segment_indices(i);
    idx_end = segment_indices(i+1);
    color_idx = mod(i-1, 2) + 1; % Alternate between 1 and 2
    if color_idx == 1 && ~left_plotted
        plot(t_out(idx_start:idx_end), log(H_tilde_out(idx_start:idx_end)), 'LineWidth', 5, ...
             'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'log($\tilde{H}_l$)');
        left_plotted = true;
    elseif color_idx == 2 && ~right_plotted
        plot(t_out(idx_start:idx_end), log(H_tilde_out(idx_start:idx_end)), 'LineWidth', 5, ...
             'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'log($\tilde{H}_r)$');
        right_plotted = true;
    else
        plot(t_out(idx_start:idx_end), log(H_tilde_out(idx_start:idx_end)), 'LineWidth', 5, ...
             'Color', hexToRGB(colors(color_idx,:)), 'HandleVisibility','off');
    end
end

% Plot events as markers with their DisplayName
plot(t_out(event_indices), log(H_tilde_out(event_indices)), 'o', 'MarkerSize', 9, ...
     'DisplayName', 'Impacts', 'Color', 'Black');

xlabel('$t\, (s)$','Interpreter','latex','FontSize', 32);
ylabel('$log(\tilde{H}_v)$', 'Interpreter','latex','FontSize', 32);
legend('show', 'Interpreter','latex','FontSize', 32, 'TextColor', labelColor);
grid on;

% Format the Figure for Publication
set(gcf, 'Color', 'w');  
set(gcf, 'Units', 'inches', 'Position', [1 1 6 6]);  % [left bottom width height]
ax = gca;
ax.XColor = labelColor;
ax.YColor = labelColor;

% Export the figure
exportgraphics(gcf, 'closed_loop_energy_pointmass.png', 'Resolution', 300);
exportgraphics(gcf, 'closed_loop_energy_pointmass.eps', 'Resolution', 300);

function x_plus = impact_mapping(x_minus,sys)
    x_plus = [x_minus(1:2);sys.P*x_minus(3:4)];
end

function u = controller(x, sys)
    q = x(1:2);
    p = x(3:4);
    % Desired configuration
    sys.qd = @(q) [q(1); 0.5 * sys.phase * sin(pi * (q(1) - sys.a) / (sys.b - sys.a))];
    sys.qtilde = @(q) q - sys.qd(q(1));
    
    % Desired momentum
    sys.vstar = @(q) sys.phase*subs(jacobian(sys.qd(sys.q),sys.q(1)),sys.q(1),q(1));
    sys.dqtildedq = @(q) subs(jacobian(sys.qtilde(sys.q),sys.q),sys.q, q);
    sys.pd = @(q) sys.M*(sys.vstar(q) - sys.K_p*sys.qtilde(q));
    sys.dpddq = @(q) subs(jacobian(sys.pd(sys.q),sys.q),sys.q,q);
    sys.ptilde = @(q,p) p - sys.pd(q);

    % Controller
    u = double(sys.G\(sys.dHdq + sys.dpddq(q)*(sys.M\p) - sys.M\(sys.dqtildedq(q).'*sys.qtilde(q)) - sys.K_d*sys.ptilde(q,p)));
end

function dx = system_dynamics(~, x, sys)
    %% Continuous Dynamics
    q = x(1:2);
    p = x(3:4);
    u = controller(x,sys);
    dx = sys.J*[sys.dHdq; sys.dHdp(p)] + [zeros(2,1); sys.G*u];
end

function [value, isterminal, direction] = collision_events(~, x,sys)
    %% Event detection for collisions    
    % Detect when q1 is equal to a or b
    value = [x(1) - sys.a; sys.b - x(1)];
    isterminal = [1; 1];  % Stop the integration
    direction = [-1; -1];  % 'a' from the right and 'b' from the left
end
