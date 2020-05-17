addpath('../src/')
addpath('../src/utils/')

% fix random seed
rng(0)

% make your own discrete linear system
A = [1 1; 0 1];
B = [0.5; 1]; 
Q = diag([1, 1]);
R = 0.1;

% constraints on state Xc and input Uc
Xc_vertex = [2, -2; 2 2; -10 2; -10 -2];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);

% construct a convex set of disturbance (2dim here)
W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
W = Polyhedron(W_vertex);

% construct disturbance Linear system
disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W)

% create a tube_mpc simulater
% if N_step is too small, the path will never reach inside the robust MPI-set X_mpi_robust in time step N_step, then the problem becomes infeasible. 
N_step = 10;
w_min = [0; -0.10];
w_max = [0; 0.10];
mpc = TubeModelPredictiveControl(disturbance_system, Xc, Uc, W, N_step);

% The robust MPC guidances the path inside the robust MPI-set so that the path will reach the robust MPI-set in N_step. After that (meaning that t > N_step), the system will be stabilized around the origin by just using LQR.
x = [-7; -2];

savedir_name = 'results'
mkdir(savedir_name)

for i = 1:15
    u_next = mpc.solve(x)
    x = disturbance_system.propagate(x, u_next)
    mpc.show_prediction()
    saveas(gcf, strcat(savedir_name, '/tmpc_seq', number2string(i), '.png'))
    clf
end

