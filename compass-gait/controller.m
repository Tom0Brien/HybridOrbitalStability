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
ctrl.vstar =  matlabFunction(dqd_sym,'vars',{sys.q_sym(1)});
ctrl.qp = @(q) [q(1);ctrl.qstar(q(1))];
ctrl.qtilde = @(q) q - ctrl.qp(q);

%% Kinetic-potential energy function
ctrl.dqtildedq = matlabFunction(jacobian(ctrl.qtilde(sys.q_sym),sys.q_sym),'vars',{sys.q_sym});
ctrl.dHtildeddq = @(q) ctrl.dqtildedq(q).'*ctrl.qtilde(q);
ctrl.pstar = @(q) sys.M(q)*(ctrl.vstar(q(1)) - Kp*eye(n)*ctrl.dHtildeddq(q));
ctrl.ptilde = @(q,p) p - ctrl.pstar(q);
ctrl.dpstarddq =  matlabFunction(jacobian(ctrl.pstar(sys.q_sym),sys.q_sym),'vars',{sys.q_sym});

%% Closed loop energy
ctrl.Hd = @(q,p) 0.5*ctrl.ptilde(q,p).'*ctrl.ptilde(q,p) + 0.5*ctrl.qtilde(q).'*ctrl.qtilde(q);

%% Control law
ctrl.u_fb = @(t,q,p) sys.G(q)\(- sys.M(q)\ctrl.dHtildeddq(q)- Kd*eye(n)*ctrl.ptilde(q,p));
ctrl.u_ff = @(t,q,p) sys.G(q)\(sys.dHdq(q,p) + sys.D(q)*sys.dHdp(q,p) + ctrl.dpstarddq(q)*(sys.M(q)\p) -sys.D(q)*ctrl.ptilde(q,p));
ctrl.u = @(t,q,p) ctrl.u_fb(t,q,p) + ctrl.u_ff(t,q,p);

end

