function [qd_sym, dqd_sym] = runSearch(run_animation)
    %% Get model
    sys = model;
    
    %% Define the search parameters
    [N,T,k] = searchParams();
    b_ij0 = 0*ones(sys.nq, k);
    b_ij0 = reshape(b_ij0,[sys.nq*k 1]);
    
    %% Define the cost function and non-linear constraints
    cost = @(b_ij) costFunc(b_ij, N, T, k, sys);
    nlcon = @(b_ij) nlCon(b_ij, T, k, sys);
    
    %% Run optimization to find optimal spline
    options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e10);
    b_ij_optimized= fmincon(cost,b_ij0,[],[],[],[],[],[],nlcon,options);
    C = reshape(b_ij_optimized,[sys.nq k]);
    
    %% Plot
    t = linspace(0,T,N);
    q = zeros(sys.nq,N);
    dq = zeros(sys.nq,N);
    Torque = zeros(2,N);
    for m=1:N
         % Compute the spline values and optimized torque at time tm
        [q(:,m), dq_m, ddq_m] = getSplineVal(C,t(m));
        Torque(:,m) = sys.Torque(q(:,m),dq_m,ddq_m);
        dq(:,m) = dq_m;
    end
    q = q.';
    dq = dq.';
    tci = length(q);
    if(run_animation)
%         animate(q,tci,sys);
        figure;
        title('Optimized q trajectory');
        plot(t,q(:,1),'LineWidth',4,'Color',"#EDB120");
        hold on;
        plot(t,q(:,2),'LineWidth',4,'Color',"#4DBEEE");
        xlabel('t', 'FontSize', 14);
        ylabel('q [rad]', 'FontSize', 14);
        legend('q1','q2', 'FontSize', 14);
        grid on;

    end

    %% Compute inital and final state
    [q_0, dq_0, ~] = getSplineVal(C,0);
    [q_T, dq_T, ~] = getSplineVal(C,T);
    x_0 = [q_0; dq_0];
    x_T = [q_T; dq_T];
    disp(x_T);
    x_plus = impactMap(x_T,sys);

    %% Fit splines to desired path and velocity as function of q1
    spline_dimension = 20;
    qd = polyfit(q(:,1),q(:,2),spline_dimension);
    dqd1 = polyfit(q(:,1),dq(:,1),spline_dimension);
    dqd2 = polyfit(q(:,1),dq(:,2),spline_dimension);
    qd_sym = poly2sym(qd,sys.q_sym(1));
    dqd_sym = [poly2sym(dqd1,sys.q_sym(1));poly2sym(dqd2,sys.q_sym(1));];
    
    %% Animate with fit spline
    if(run_animation)
        figure;
%         subplot(3,1,1)
        plot(q(:,1),q(:,2),'LineWidth',4,'Color','blue');
        grid on;
        hold on;
%         plot(q(:,1),polyval(qd,q(:,1)),'--','LineWidth',4,'Color','blue');
        hold on;
        plot([-0.2,-0.3],[0.2,0.3],'--','LineWidth',4,'Color','black');
        xlabel('$q_1$','FontSize', 14,'Interpreter','latex');
        ylabel('$q_2$','FontSize', 14,'Interpreter','latex');
        legend('$q_p(q_1)$ desired path','Switching Surface $\mathcal{S}$','FontSize', 14,'Interpreter','latex');
        xlim([-0.3 0.25268])
        title('Desired Path Generation')
%         subplot(3,1,2)
%         plot(q(:,1),dq(:,1),'LineWidth',4,'Color','black');
%         grid on;
%         hold on;
%         plot(q(:,1),polyval(dqd1,q(:,1)),'--','LineWidth',4,'Color',"#EDB120");
%         xlabel('$q_1$','FontSize', 14,'Interpreter','latex');
%         ylabel('$\dot{q}_1$','FontSize', 14,'Interpreter','latex');
%         legend('$\dot{q}_1(q_1)$','FontSize', 14,'Interpreter','latex');
%         subplot(3,1,3)
%         plot(q(:,1),dq(:,2),'LineWidth',4,'Color','black');
%         grid on;
%         hold on;
%         plot(q(:,1),polyval(dqd2,q(:,1)),'--','LineWidth',4,'Color',"#4DBEEE");
%         xlabel('$q_1$','FontSize', 14,'Interpreter','latex');
%         ylabel('$\dot{q}_2$','FontSize', 14,'Interpreter','latex');
%         legend('$\dot{q}_2(q_1)$','FontSize', 14,'Interpreter','latex');
    end
end
