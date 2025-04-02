function sys = model()
%% Symbolic variables
syms q1 q2 p1 p2
sys.q_sym = [q1; q2];
sys.p_sym = [p1; p2];

%% System dimensions
sys.nq = 2; 

%% Input mapping matrix
sys.G = @(q) [1 0; 1 -1];

%% Mass matrix
sys.mH = 10;
sys.m = 5;
sys.l = 1;
sys.a = 0.5;
sys.b = 0.5;

sys.m11 = @(q) (sys.mH+sys.m)*sys.l^2 + sys.m*sys.a^2;
sys.m12 = @(q) -sys.m*sys.l*sys.b*cos(q(1)-q(2));
sys.m21 = @(q) -sys.m*sys.l*sys.b*cos(q(1)-q(2));
sys.m22 = @(q) sys.m*sys.b^2;
sys.M = @(q) [sys.m11(q) sys.m12(q); sys.m21(q), sys.m22(q)];

%% Coriolis matrix 
c11 = @(q,dq) 0;
c12 = @(q,dq) -sys.m*sys.l*sys.b*sin(q(1) - q(2))*dq(2);
c21 = @(q,dq) sys.m*sys.l*sys.b*sin(q(1) - q(2))*dq(1);
c22 = @(q,dq) 0;
sys.C = @(q,dq) [c11(q,dq) c12(q,dq); c21(q,dq), c22(q,dq)];

%% Gravitational torque vector
sys.gravity = 9.81;
g1 = @(q) -(sys.mH*sys.l + sys.m*sys.a + sys.m*sys.l)*sys.gravity*sin(q(1));
g2 = @(q) sys.m*sys.b*sys.gravity*sin(q(2));
sys.g = @(q) [g1(q);g2(q)];
            
%% Friction parameters
sys.D = @(q) zeros(2); 

%% Potential parameters
sys.V = @(q) (sys.m*(sys.a + sys.l) + sys.mH)*sys.gravity*cos(q(1)) - sys.m*sys.b*sys.gravity*cos(q(2));

%% System energy
sys.H = @(q,p) 0.5*p.'*(sys.M(q)\p) + sys.V(q);
sys.T = @(q,p) 0.5*p.'*(sys.M(q)\p);

%% Energy gradients
sys.dVdq = matlabFunction(jacobian(sys.V(sys.q_sym),sys.q_sym).','vars',{sys.q_sym});
sys.dHdq = matlabFunction(jacobian(sys.H(sys.q_sym,sys.p_sym),sys.q_sym).','vars',{sys.q_sym, sys.p_sym});
sys.dHdp = @(q,p) sys.M(q)\p;

%% Torque
sys.Torque = @(q,dq,ddq) sys.M(q)*ddq + sys.C(q,dq)*dq + sys.g(q);

%% Ground slope
sys.psi = 0;

%% Forward Kinematics
sys.x = @(q) sys.l*(sin(-q(1) - sys.psi) + sin(q(2) + sys.psi));
sys.y = @(q) sys.l*(cos(-q(1) - sys.psi) - cos(q(2) + sys.psi));
end

