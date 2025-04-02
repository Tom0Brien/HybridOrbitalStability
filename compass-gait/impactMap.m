function xplus = impactMap(xminus, sys)
%% States pre-impact
q_minus = [xminus(1);xminus(2)];
dq_minus = [xminus(3);xminus(4)];
% Coordinate Transformation Method
M = [20,              0, 2.5000*cos(q_minus(1)), 2.5000*cos(q_minus(2));
             0,             20, 2.5000*sin(q_minus(1)), 2.5000*sin(q_minus(2));
2.5000*cos(q_minus(1)), 2.5000*sin(q_minus(1)),         1.2500,              0;
2.5000*cos(q_minus(2)), 2.5000*sin(q_minus(2)),              0,         1.2500];

Q_minus = [-cos(q_minus(1)), 0;
           -sin(q_minus(1)), 0;
           1, 0;
           0, 1];

Q_plus =  [0, -cos(q_minus(2));
           0, -sin(q_minus(2));
           1,        0;
           0,        1];

% Relabelling matrix
S = [0,1;1,0];
q_plus = S*q_minus;

% Velocity Projection Operator
dq_plus = (S*sys.M(q_plus))\(Q_plus.'*M*Q_minus)*dq_minus;

xplus = [q_plus; dq_plus];
end

