addpath('../src/')
addpath('../src/utils/')

% make your own discrete linear system
A = [1 1; 0 1];
B = [0.5; 1]; 
Q = diag([1, 1]);
R = 0.1;
mysys = LinearSystem(A, B, Q, R);

% constraints on state Xc and input Uc
Xc_vertex = [2, -2; 2 2; -10 2; -10 -2];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);

% construct a convex set of system noise (2dim here)
W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
W = Polyhedron(W_vertex);

% create a tube_mpc simulater
% if N_step is too small, the path will never reach inside the robust MPI-set (X_MPI - Z) in time step N_step, then the problem becomes infeasible. 
% Please note that rectangle defined by w_min and w_max must be included in W, otherwise, of course, the robustness is not guranteed.
N_step = 10;
w_min = [0; -0.10];
w_max = [0; 0.10];
mpc = TubeModelPredictiveControl(mysys, Xc, Uc, W, N_step, w_min, w_max);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set exactly at N_step. After that (meaning that t > N_step), the system will be stabilized around the origin by just using LQR.
T_simu = 1
x = [-7; -2];



for i = 1:T_simu
    u_next = mpc.solve(x)
    x = mysys.propagate(x, u_next)
    mpc.show_prediction()
end
