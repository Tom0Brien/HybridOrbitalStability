clear;clc;close all;
%% Build system model for compass-gait walker
sys = model;

%% Run trajectory optimisation routine to design walking gait
[qd_sym, dqd_sym] = runSearch(sys);

%% Build Controller
ctrl = controller(sys,1*eye(sys.nq),8*eye(sys.nq),qd_sym,dqd_sym);

%% Simulation Parameters
sim = simParams();
sim.dx = @(q,dq,u) [dq;sys.M(q)\(-sys.C(q,dq)*dq - sys.g(q) - sys.D(q)*dq + sys.G(q)*u)];
sim.ode = @(t,x) sim.dx(x(1:2),x(3:4),ctrl.u(t,x(1:2),sys.M(x(1:2))*x(3:4)));
% Impact event and mapping
impactMapping = @(x) impactMap(x,sys);
impactEventDetection = @(t,x) impactEvent(t,x,sys);
sim.options = odeset('Events', impactEventDetection,'RelTol',1e-9,'AbsTol',1e-9);

%% Run simulation
res.T = []; % Time history
res.X = []; % State history
res.impact_idx = []; % Impact time history
while(sim.current_time < sim.total_sim_time ) && (sim.step_count < sim.max_steps)
    [res.t,res.x] = ode45(sim.ode,sim.tspan,sim.x0, sim.options);
    sim.x0 = impactMapping(res.x(end,:).');
    fprintf('Swing foot impact at time = %0.2f\n', res.t(end));
    res.T = [res.T; res.t];
    res.X = [res.X; res.x];
    res.impact_idx = [res.impact_idx length(res.T)];
    sim.tspan = [res.t(end) max(sim.total_sim_time, res.T(end))];
    current_time = res.t(end);
    sim.step_count = sim.step_count + 1;
end

%% Plot results and animate
plotResults(sys,ctrl,sim,res);
animate(res.X,res.impact_idx,sys);