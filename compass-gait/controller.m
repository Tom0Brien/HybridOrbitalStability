function ctrl = controller(sys,Kp,Kd,qd_sym,dqd_sym)
%% Desired path
ctrl.qbar =  matlabFunction(qd_sym,'vars',{sys.q_sym(1)});
ctrl.vstar =  matlabFunction(dqd_sym,'vars',{sys.q_sym(1)});
ctrl.qstar = @(q) [q(1);ctrl.qbar(q(1))];
ctrl.qtilde = @(q) q - ctrl.qstar(q);

%% Control law
ctrl.dqtildedq = matlabFunction(jacobian(ctrl.qtilde(sys.q_sym),sys.q_sym),'vars',{sys.q_sym});
ctrl.dHtildeddq = @(q) ctrl.dqtildedq(q).'*ctrl.qtilde(q);
ctrl.pstar = @(q) sys.M(q)*(ctrl.vstar(q(1)) - Kp*ctrl.dHtildeddq(q));
ctrl.ptilde = @(q,p) p - ctrl.pstar(q);
ctrl.dpstarddq =  matlabFunction(jacobian(ctrl.pstar(sys.q_sym),sys.q_sym),'vars',{sys.q_sym});
ctrl.u_fb = @(t,q,p) sys.G(q)\(-sys.M(q)\ctrl.dHtildeddq(q)- Kd*ctrl.ptilde(q,p));
ctrl.u_ff = @(t,q,p) sys.G(q)\(sys.dHdq(q,p) + sys.D(q)*sys.dHdp(q,p) + ctrl.dpstarddq(q)*(sys.M(q)\p) -sys.D(q)*ctrl.ptilde(q,p));
ctrl.u = @(t,q,p) ctrl.u_fb(t,q,p) + ctrl.u_ff(t,q,p);


%% Closed-loop energy
ctrl.Hd = @(q,p) 0.5*ctrl.ptilde(q,p).'*ctrl.ptilde(q,p) + 0.5*ctrl.qtilde(q).'*ctrl.qtilde(q);
end

