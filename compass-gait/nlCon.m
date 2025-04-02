function [c,ceq] = nlCon(b_ij, T, k, sys)
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