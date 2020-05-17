addpath('../src/')
addpath('../src/utils/')

%the usage is mostly same as tubeMPC
A = [1 1; 0 1];
B = [0.5; 1]; 
Q = diag([1, 1]);
R = 0.1;
mysys = LinearSystem(A, B, Q, R);

Xc_vertex = [2, -2; 2 2; -10 2; -10 -2];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);

% Unlike tube-MPC, this generic MPC doesn't guarantee the robustness. 
% In the following simulation you will notice that the generated path almost touch the boundary.
% Thus, you can imagine that if some additive disturbance is considered, then the state can 
% easily be off the boundary, and the succeeding optimization will no longer be feasible.
% Please add some noise and do experiments.
N_horizon = 5;
mpc = ModelPredictiveControl(mysys, Xc, Uc, N_horizon);

x = [-7; -2];
savedir_name = 'results';
for i = 1:15
    u_next = mpc.solve(x);
    x = mysys.propagate(x, u_next); % + add some noise here
    mpc.show_prediction();
    pause(0.1);
    clf;
end

