function plotResults(sys, ctrl, sim, res)
    % Define the new color scheme for alternating segments
    colors = ['#a5d0ea'; '#c2e5ce']; % Light blue and light green
    hexToRGB = @(hex) sscanf(hex(2:end), '%2x')' / 255;
    labelColor = '#545454'; % Grey for axes 
    close all;
    %% Unpack results
    N = length(res.T);
    res.q  = res.X(:,1:2);
    res.p  = zeros(N,2);
    res.H  = zeros(N,1);
    res.Hd = zeros(N,1);
    res.qd = zeros(N,2);
    res.qe = zeros(N,2);
    res.vstar = zeros(N,2);
    res.qp  = zeros(N,2);
    
    for i = 1:N
        res.p(i,:) = sys.M(res.q(i,:)) * res.X(i,3:4).';
        res.H(i)  = sys.H(res.q(i,:).', res.p(i,:).');
        res.Hd(i) = ctrl.Hd(res.q(i,:).', res.p(i,:).');
        res.qp(i,:) = ctrl.qp(res.q(i,:)).';
        res.qe(i,:) = res.q(i,:) - res.qp(i,:);
        res.vstar(i,:) = ctrl.vstar(res.q(i,1));
    end
    all_indices = res.impact_idx;
    if isempty(all_indices) || all_indices(end) < N
        all_indices = [all_indices, N];
    end
    start_idx = 1;
    for seg = 1:length(all_indices)
        idx_range = start_idx:all_indices(seg);
        if mod(seg,2) == 0
            res.q(idx_range,:) = res.q(idx_range, [2,1]);
            res.p(idx_range,:) = res.p(idx_range, [2,1]);
            res.qp(idx_range,:) = res.qp(idx_range, [2,1]);
        end
        start_idx = all_indices(seg) + 1;
    end
    
    %% Figure 1: Closed-Loop Coordinates vs. Time
    T_end = res.T(find(any(res.X,2), 1, 'last'));
    validIdx = res.T <= T_end;
    t_valid = res.T(validIdx);
    qp_valid = res.qp(validIdx,:);
    dqd_valid = res.vstar(validIdx,:);
    
    figure(1);
    segment_indices = [1, all_indices];
    num_segments = length(segment_indices) - 1;
    
    %% Subplot 1: q3
    subplot(4,1,1);
    hold on;
    leftLegendPlotted = false;
    rightLegendPlotted = false;
    for i = 1:num_segments
        idx_start = segment_indices(i);
        idx_end = segment_indices(i+1);
        color_idx = mod(i-1, 2) + 1;
        valid_segment = idx_start:min(idx_end, length(t_valid));
        if color_idx == 1 && ~leftLegendPlotted
            plot(t_valid(valid_segment), res.q(valid_segment,1), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'Actual $v_l$');
            leftLegendPlotted = true;
        elseif color_idx == 2 && ~rightLegendPlotted
            plot(t_valid(valid_segment), res.q(valid_segment,1), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'Actual $v_r$');
            rightLegendPlotted = true;
        else
            plot(t_valid(valid_segment), res.q(valid_segment,1), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'HandleVisibility','off');
        end
    end
    plot(t_valid, qp_valid(:,1), '--', 'LineWidth', 4, 'Color', labelColor, ...
         'DisplayName', 'Reference');
    formatAxis(labelColor);
    xlim([0 2.5]);  % Set x-axis limits to [0, 2.5]
    ylabel('$q_3$', 'Interpreter', 'latex');
    
    %% Subplot 2: q4
    subplot(4,1,2);
    hold on;
    leftLegendPlotted = false;
    rightLegendPlotted = false;
    for i = 1:num_segments
        idx_start = segment_indices(i);
        idx_end = segment_indices(i+1);
        color_idx = mod(i-1, 2) + 1;
        valid_segment = idx_start:min(idx_end, length(t_valid));
        if color_idx == 1 && ~leftLegendPlotted
            plot(t_valid(valid_segment), res.q(valid_segment,2), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'Actual $v_l$');
            leftLegendPlotted = true;
        elseif color_idx == 2 && ~rightLegendPlotted
            plot(t_valid(valid_segment), res.q(valid_segment,2), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'Actual $v_r$');
            rightLegendPlotted = true;
        else
            plot(t_valid(valid_segment), res.q(valid_segment,2), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'HandleVisibility','off');
        end
    end
    plot(t_valid, qp_valid(:,2), '--', 'LineWidth', 4, 'Color', labelColor, ...
         'DisplayName', 'Reference');
    formatAxis(labelColor);
    xlim([0 2.5]);  % Set x-axis limits to [0, 2.5]
    ylabel('$q_4$', 'Interpreter', 'latex');
    
    %% Subplot 3: dq3
    subplot(4,1,3);
    hold on;
    leftLegendPlotted = false;
    rightLegendPlotted = false;
    for i = 1:num_segments
        idx_start = segment_indices(i);
        idx_end = segment_indices(i+1);
        color_idx = mod(i-1, 2) + 1;
        valid_segment = idx_start:min(idx_end, length(t_valid));
        if color_idx == 1 && ~leftLegendPlotted
            plot(t_valid(valid_segment), res.X(valid_segment,3), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'Actual $v_l$');
            leftLegendPlotted = true;
        elseif color_idx == 2 && ~rightLegendPlotted
            plot(t_valid(valid_segment), res.X(valid_segment,3), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'Actual $v_r$');
            rightLegendPlotted = true;
        else
            plot(t_valid(valid_segment), res.X(valid_segment,3), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'HandleVisibility','off');
        end
    end
    plot(t_valid, dqd_valid(:,1), '--', 'LineWidth', 4, 'Color', labelColor, ...
         'DisplayName', 'Reference');
    formatAxis(labelColor);
    xlim([0 2.5]);  % Set x-axis limits to [0, 2.5]
    ylabel('$\dot{q}_3$', 'Interpreter', 'latex');
    
    %% Subplot 4: dq4 (with legend)
    subplot(4,1,4);
    hold on;
    leftLegendPlotted = false;
    rightLegendPlotted = false;
    for i = 1:num_segments
        idx_start = segment_indices(i);
        idx_end = segment_indices(i+1);
        color_idx = mod(i-1, 2) + 1;
        valid_segment = idx_start:min(idx_end, length(t_valid));
        if color_idx == 1 && ~leftLegendPlotted
            plot(t_valid(valid_segment), res.X(valid_segment,4), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'Actual $v_l$');
            leftLegendPlotted = true;
        elseif color_idx == 2 && ~rightLegendPlotted
            plot(t_valid(valid_segment), res.X(valid_segment,4), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'DisplayName', 'Actual $v_r$');
            rightLegendPlotted = true;
        else
            plot(t_valid(valid_segment), res.X(valid_segment,4), 'LineWidth', 5, ...
                 'Color', hexToRGB(colors(color_idx,:)), 'HandleVisibility','off');
        end
    end
    plot(t_valid, dqd_valid(:,2), '--', 'LineWidth', 4, 'Color', labelColor, ...
         'DisplayName', 'Reference');
    formatAxis(labelColor);
    xlim([0 2.5]);
    xlabel('$t\, (s)$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
    ylabel('$\dot{q}_4$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
    
    % Add legend only for the last subplot
    legend('show', 'Interpreter', 'latex', 'FontSize', 16, 'TextColor', labelColor,'Location','northeast');
    
    saveFigure(1, 'closed_loop_coordinates', 6, 6);
    
    %% Figure 4: Log Closed-Loop Energy with Color-Matched Impact Markers
    figure(2);
    hold on;
    
    % Define styling parameters
    colors = ['#a5d0ea'; '#c2e5ce']; % Segment colors
    light_grey = [0.8 0.8 0.8];      % Connection line color
    hexToRGB = @(hex) sscanf(hex(2:end), '%2x')'/255;
    labelColor = '#545454';          % Axis text color
    
    % Get segment boundaries from impact indices
    all_indices = res.impact_idx;
    if isempty(all_indices) || all_indices(end) < length(res.T)
        all_indices = [all_indices, length(res.T)];
    end
    segments = arrayfun(@(x,y) x:y, [1 all_indices(1:end-1)+1], all_indices, 'UniformOutput', false);
    
    % Plot each segment with markers and connections
    for i = 1:length(segments)
        % Current segment properties
        seg = segments{i};
        color_idx = mod(i-1, 2) + 1;
        seg_color = hexToRGB(colors(color_idx,:));
        
        % Plot main segment trajectory
        plot(res.T(seg), log(res.Hd(seg)), 'LineWidth', 5, 'Color', seg_color, ...
            'DisplayName', sprintf('Segment %d', i));
        
        % Add pre-impact marker (end of current segment)
        if i < length(segments)
            plot(res.T(seg(end)), log(res.Hd(seg(end))), 'o', ...
                'MarkerSize', 9, 'MarkerEdgeColor', seg_color, ...
                'MarkerFaceColor', 'none', 'HandleVisibility', 'off');
        end
        
        % Add post-impact marker (start of next segment)
        if i > 1
            plot(res.T(seg(1)), log(res.Hd(seg(1))), 'o', ...
                'MarkerSize', 9, 'MarkerEdgeColor', seg_color, ...
                'MarkerFaceColor', seg_color, 'HandleVisibility', 'off');
        end
        
        % Connect segments with dashed line
        if i < length(segments)
            next_seg = segments{i+1}(1);
            plot([res.T(seg(end)), res.T(next_seg)], ...
                [log(res.Hd(seg(end))), log(res.Hd(next_seg))], ...
                '--', 'Color', light_grey, 'LineWidth', 2, 'HandleVisibility', 'off');
        end
    end
    
    % Format final plot
    grid on;
    xlabel('$t\, (s)$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
    ylabel('$\log(\tilde{H})$', 'Interpreter', 'latex', 'FontSize', 22, 'Color', labelColor);
    legend({'$\log(\tilde{H}_l)$', '$\log(\tilde{H}_r)$'}, 'Interpreter', 'latex', ...
        'FontSize', 16, 'TextColor', labelColor, 'Location', 'best');
    xlim([0 2.5]);
    % Set figure properties
    set(gcf, 'Color', 'w', 'Units', 'inches', 'Position', [1 1 6 6]);
    saveFigure(2, 'log_closed_loop_energy', 6, 6);
end

function formatAxis(labelColor)
    ax = gca;
    ax.XColor = labelColor;
    ax.YColor = labelColor;
    ax.FontSize = 18;
    grid on;
    xlim([ax.XAxis.Limits]);
end

function saveFigure(figNum, fileName, width, height)
    set(gcf, 'Color', 'w', 'Units', 'inches', 'Position', [1 1 width height]);
    exportgraphics(gcf, ['results/' fileName '.png'], 'Resolution', 300);
    exportgraphics(gcf, ['results/' fileName '.pdf'], 'ContentType', 'vector');
    exportgraphics(gcf, ['results/' fileName '.eps'], 'ContentType', 'vector');
end
