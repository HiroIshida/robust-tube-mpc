addpath('../src/')
addpath('../src/utils/')

% make your own discrete linear system
A = [1 1; 0 1];
B = [0.5; 1]; 
Q = diag([1, 1]);
R = 0.1;
mysys = LinearSystem(A, B, Q, R); 

% constraints on 2dim state Xc and 1dim input Uc
Xc_vertex = [2, -2; 2 2; -10 2; -10 -2];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);

% create a optimal controler
% if N_step is too small, the path will never reach inside 
% the maximum positively invariant set (X_MPI) in time step N_step, 
% then the problem becomes infeasible.
% OptimalControler.solve method returns optimal path (x_seq) and input (u_seq)
N_step = 20; 
optcon = OptimalControler(mysys, Xc, Uc, 20);
x_init = [-7; -2];
optcon.add_initial_eq_constraint(x_init)
[x_seq, u_seq] = optcon.solve();
