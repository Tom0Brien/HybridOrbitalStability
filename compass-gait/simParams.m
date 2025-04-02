function sim = simParams()
    sim.total_sim_time = 20;
    sim.tspan = [0 sim.total_sim_time]; 
    sim.impact_idx = [];
    sim.current_time = 0;
    sim.step_count = 0;
    sim.max_steps = 10;
    sim.t_end = 4;
    sim.tspan = [0 sim.t_end];
    sim.q0 = 1.2*[0.2527;-0.252668056151167]; % Introduce some inital error
    sim.dq0 = [-1.323855316770684;-0.833241648151501];
    sim.x0 = [sim.q0; sim.dq0];
end

