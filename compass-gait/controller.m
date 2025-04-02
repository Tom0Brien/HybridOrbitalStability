function ctrl = controller(sys,Kp,Kd,qd_sym,dqd_sym)
%% CONTROLLER
% Generates a controller for a 2DOF manipulator with user provided gains 
% Kp, Kd and K_alpha

%% Symbolic variables
syms z1 z2 
z_sym = [z1 z2].';

%% System dimensions
n = sys.nq;

%% Desired path
ctrl.qstar =  matlabFunction(qd_sym,'vars',{sys.q_sym(1)});
ctrl.dqd =  matlabFunction(dqd_sym,'vars',{sys.q_sym(1)});
ctrl.fzb = @(q) [q(2) - ctrl.qstar(q(1))];
ctrl.qp = @(q) [q(1);ctrl.qstar(q(1))];

ctrl.qtilde = @(q) [0;q(2) - ctrl.qstar(q(1))];

%% Kinetic-potential energy function
% ctrl.Kp = Kp*eye(n-1);
ctrl.Vd = @(q) 0.5*ctrl.qtilde(q).'*ctrl.qtilde(q);

%% Vector field
ctrl.dfzbdq = matlabFunction(jacobian(ctrl.qtilde(sys.q_sym),sys.q_sym),'vars',{sys.q_sym});
ctrl.dVddq = @(q) ctrl.dfzbdq(q).'*ctrl.qtilde(q);
ctrl.K_p = Kp*eye(n);
ctrl.vdb = @(q) ctrl.dqd(q(1)) - ctrl.K_p*ctrl.dVddq(q);

%% Momentum error coordinates
ctrl.pd = @(q) sys.M(q)*ctrl.vdb(q);
ctrl.ptilde = @(q,p) p - ctrl.pd(q);
ctrl.dpddq =  matlabFunction(jacobian(ctrl.pd(sys.q_sym),sys.q_sym),'vars',{sys.q_sym});

%% Closed loop energy
ctrl.Hd = @(q,p) 0.5*ctrl.ptilde(q,p).'*ctrl.ptilde(q,p) + ctrl.Vd(q);

%% Control law
ctrl.Kd = Kd*eye(n);
ctrl.ubar = @(t,q,p) - ctrl.Kd*ctrl.ptilde(q,p) - sys.M(q)\ctrl.dVddq(q);
ctrl.u = @(t,q,p) sys.G(q)\(sys.dHdq(q,p) + sys.D(q)*sys.dHdp(q,p) + ctrl.dpddq(q)*(sys.M(q)\p) -sys.D(q)*ctrl.ptilde(q,p) + ctrl.ubar(t,q,p));

end

