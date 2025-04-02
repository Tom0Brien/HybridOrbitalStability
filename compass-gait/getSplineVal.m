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