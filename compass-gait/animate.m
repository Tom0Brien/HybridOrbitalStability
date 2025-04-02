function animate(q,tci,p)
    % Record Figure
    writerObj = VideoWriter('results/passive_walk');
    writerObj.FrameRate = 60;
    writerObj.Quality = 100;
    open(writerObj);

    % Animate passive dynamic walking data
    % Position of stance foot
    rAAa = [0;0];
    % Position of hip
    rBAa = [rAAa(1) + p.l*sin(-q(1,1)); rAAa(2) + p.l*cos(-q(1,1))];
    % Position of swing foot
    rCBa = rBAa - [p.l*sin(-q(1,2)); p.l*cos(-q(1,2))];

    % Initialize figure for animation
    figure('Color','w','Renderer','zbuffer')
    axis([0 5 0 1.1])
    set(gca,'nextplot','replacechildren');
    % Fix the figure size to 1000x200
    set(gcf, 'Position', [100, 100, 1280, 240]);
    set(gcf,'Renderer','zbuffer');

    % Plot slope
    R = @(slope) [cos(slope), sin(slope); -sin(slope), cos(slope)];
    rPAa = R(p.psi)*[100;0];
    rPAa2 = R(p.psi)*[-100;0];
    slope = line([rPAa2(1) rPAa(1)], [rPAa2(2) rPAa(2)]);
    set(slope,'Color','#545454','LineWidth',4)

    % Draw first position
    stanceColor = '#a5d0ea';  % Light blue
    swingColor  = '#c2e5ce';  % Light green

    stleg = line([rAAa(1) rBAa(1)], [rAAa(2) rBAa(2)]);
    set(stleg,'Color',stanceColor,'LineWidth',2);
    swleg = line([rBAa(1) rCBa(1)], [rBAa(2) rCBa(2)]);
    set(swleg,'Color',swingColor,'LineWidth',4);
    hold on;

    hip   = plot(rBAa(1), rBAa(2), 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'blue');
    lfoot = plot(rAAa(1), rAAa(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'white', 'MarkerEdgeColor', '#545454', 'LineWidth', 2);
    rfoot = plot(rCBa(1), rCBa(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'white', 'MarkerEdgeColor', '#545454', 'LineWidth', 2);
    tci = [0, tci];
    rAAa = [0;0]; % Reset stance foot

    % Animate each stride
    for i = 1:length(tci)-1
        for j = tci(i)+1 : 10 : tci(i+1)
            % Position of hip
            rBAa = [rAAa(1) + p.l*sin(-q(j,1)); rAAa(2) + p.l*cos(-q(j,1))];
            % Position of swing foot
            rCBa = rBAa - [p.l*sin(-q(j,2)); p.l*cos(-q(j,2))];

            % Clear previous legs
            set([stleg swleg hip lfoot rfoot],'Visible','off');

            % Plot new leg positions with updated colors
            stleg = line([rAAa(1) rBAa(1)], [rAAa(2) rBAa(2)]);
            set(stleg,'Color',stanceColor,'LineWidth',8);
            swleg = line([rBAa(1) rCBa(1)], [rBAa(2) rCBa(2)]);
            set(swleg,'Color',swingColor,'LineWidth',8);
            hold on;

            hip   = plot(rBAa(1), rBAa(2), 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'white', 'MarkerEdgeColor', '#545454', 'LineWidth', 2);
            lfoot = plot(rAAa(1), rAAa(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'white', 'MarkerEdgeColor', '#545454', 'LineWidth', 2);
            rfoot = plot(rCBa(1), rCBa(2), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'white', 'MarkerEdgeColor', '#545454', 'LineWidth', 2);

            drawnow

            % Capture and resize the frame for the video
            frame = getframe(gcf);
            writeVideo(writerObj, frame);
        end

        % Add light trace every 2 steps
        if (mod(j,2) == 0)
            stleg_hist = line([rAAa(1) rBAa(1)], [rAAa(2) rBAa(2)]);
            set(stleg_hist,'Color',[0 0 0 0.2],'LineWidth',3,'LineStyle','--');
            swleg_hist = line([rBAa(1) rCBa(1)], [rBAa(2) rCBa(2)]);
            set(swleg_hist,'Color',[0 0 0 0.2],'LineWidth',3,'LineStyle','--');
            hold on;
            hip_hist = plot(rBAa(1), rBAa(2), 'o', 'MarkerSize', 15, 'MarkerFaceColor', 'white', ...
                            'MarkerEdgeColor', [200 200 200]/255, 'LineWidth', 1);
        end

        % Update stance foot location
        rAAa = rCBa;
    end

    close(writerObj);
end
