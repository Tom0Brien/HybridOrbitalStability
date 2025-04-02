%% Cost function and constraints
function cost = costFunc(b_ij, N, T, k, sys)
    cost = 0;
    % Convert b_ij to a square matrix
    C = reshape(b_ij,[sys.nq k]);
    % For each time point from 0 to T, add up the sum of the torques from
    % the spline based q(tm), dq(tm), ddq(tm) 
    t = linspace(0,T,N);
    for m=1:N
        % Compute the spline values at time tm
        [q_m, dq_m, ddq_m] = getSplineVal(C,t(m));
        % Compute the torque 
        Torque = sys.Torque(q_m,dq_m,ddq_m);
        % Add to cost
        cost = cost + Torque.'*eye(sys.nq)*Torque;
    end
end