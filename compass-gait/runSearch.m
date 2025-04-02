function [qd_sym, dqd_sym] = runSearch(sys)
    %% Define the search parameters
    N = 25;
    T = 0.5;
    k = 5;
    b_ij0 = zeros(sys.nq, k);
    b_ij0 = reshape(b_ij0,[sys.nq*k 1]);
    
    %% Define the cost function and non-linear constraints
    cost = @(b_ij) costFunction(b_ij, N, T, k, sys);
    nlcon = @(b_ij) nlConstraints(b_ij, T, k, sys);
    
    %% Run optimization to find optimal spline
    options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e10);
    b_ij_optimized= fmincon(cost,b_ij0,[],[],[],[],[],[],nlcon,options);
    C = reshape(b_ij_optimized,[sys.nq k]); % fmincon requires optimisation params to be a vector

    %% Convert time-parameterised trajecory to path parameterised, with phase variable q1
    t = linspace(0,T,N);
    q = zeros(sys.nq,N);
    dq = zeros(sys.nq,N);
    for m=1:N
        [q(:,m), dq(:,m), ~] = getSplineVal(C,t(m));
    end
    q = q.';
    dq = dq.';

    spline_dimension = 10;
    qd = polyfit(q(:,1),q(:,2),spline_dimension);
    dqd1 = polyfit(q(:,1),dq(:,1),spline_dimension);
    dqd2 = polyfit(q(:,1),dq(:,2),spline_dimension);
    qd_sym = poly2sym(qd,sys.q_sym(1));
    dqd_sym = [poly2sym(dqd1,sys.q_sym(1));poly2sym(dqd2,sys.q_sym(1));];
end

%% Cost function and constraints
function total_cost = costFunction(b_ij, N, T, k, sys)
    total_cost = 0;
    % Convert b_ij to a square matrix
    C = reshape(b_ij,[sys.nq k]);
    % For each time point from 0 to T, add up the sum of the torques
    t = linspace(0,T,N);
    for m=1:N
        % Compute the spline values at time tm
        [q_m, dq_m, ddq_m] = getSplineVal(C,t(m));
        total_cost = total_cost + sys.Torque(q_m,dq_m,ddq_m).'*eye(sys.nq)*sys.Torque(q_m,dq_m,ddq_m);
    end
end

function [c,ceq] = nlConstraints(b_ij, T, k, sys)
    % Reshape param vector into correct shape
    C = reshape(b_ij,[sys.nq k]);

    % Compute inital and final state
    [q_0, dq_0, ~] = getSplineVal(C,0);
    x_0 =  [q_0; dq_0];
    [q_T, dq_T, ~] = getSplineVal(C,T);
    x_T = [q_T; dq_T];

    % Impact mapping
    x_plus = impactMap(x_T,sys);
    
    % Constraints 
    ceq = [x_0 - x_plus;  % Periodicity constraint
           sys.y(q_T)];   % Swing foot landing on ground after step

    c = 1 - (sys.x(q_T)-sys.x(q_0)); % Ensure swing foot is greater than 1m away from starting position
end

function [qij_m, dqij_m, ddqij_m] = getSplineVal(C, tm)
        % Computes q, dq, and ddq for spline at time tm
        % C : The spline coefficents      
        k = size(C,2);
        % Compute the time vector
        tj = tm.^(0:k-1).';
        qij_m = C*tj;
        % Derivative operator
        D = diag(1:k-1, -1);
        dqij_m = C*D*tj;
        ddqij_m = C*D^2*tj;
end
