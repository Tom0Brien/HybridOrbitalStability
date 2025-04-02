clear;clc;close all;
%% Run gait search
run_animation = true;
[qd_sym, dqd_sym] = runSearch(run_animation);
save('qd_sym.mat');
save('dqd_sym.mat');
%% Build system model for 2DOF manipulator
sys = model();

%% Build Controller
ctrl = controller(sys,1,8,qd_sym,dqd_sym);

%% Define impact event and mapping
% Impact map
impactMapping = @(x) impactMap(x,sys);
% Impact event
impactEventDetection = @(t,x) impactEvent(t,x,sys);

%% Simulation Parameters
sim = simParams();
% System ODE
% sim.dx = @(q,p,u) [zeros(2) eye(2); -eye(2) -sys.D(q)]*[sys.dHdq(q,p); sys.dHdp(q,p)] + [zeros(2); sys.G(q)]*u; 
sim.dx = @(q,dq,u) [dq;sys.M(q)\(-sys.C(q,dq)*dq - sys.g(q) - sys.D(q)*dq + sys.G(q)*u)];
sim.ode = @(t,x) sim.dx(x(1:2),x(3:4),ctrl.u(t,x(1:2),sys.M(x(1:2))*x(3:4)));
sim.options = odeset('Events', impactEventDetection,'RelTol',1e-9,'AbsTol',1e-9);

%% Run simulation
while(sim.current_time < sim.total_sim_time ) && (sim.step_count < sim.max_steps)
    [res.t,res.x] = ode45(sim.ode,sim.tspan,sim.x0, sim.options);
    % Apply impact mapping
    sim.x0 = impactMapping(res.x(end,:).');
    fprintf('Swing foot impact at time = %0.2f\n', res.t(end));
    sim.T = [sim.T; res.t];
    sim.X = [sim.X; res.x];
    sim.impact_idx = [sim.impact_idx length(sim.T)];
    sim.tspan = [res.t(end) max(sim.total_sim_time, sim.T(end))];
    current_time = res.t(end);
    sim.step_count = sim.step_count + 1;
end

%% Plot results and figures
res.t = sim.T;
res.x = sim.X;
plot_results(sys,ctrl,sim,res);

%% Animate results of simulation
if(run_animation)
    animate(res.x,sim.impact_idx,sys);
end