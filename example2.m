% reachability set analysis

A = [1 1; 0 1]; B = [0.5; 1]; 
Q = diag([1, 1]); R = 0.1;
%Xc_vertex = [2, -2; 2 2; -10 2; -10 -2];
Xc_vertex = [10, 10; 10, -10; -10, -10; -10, 10];
Uc_vertex = [1; -1];
W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
W = Polyhedron(W_vertex);

x_init = [-7; -2]; N = 10;
clf;
mp = TubeModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N, x_init);
F0 = mp.Xmpi;
F0.show(); hold on;
F1 = inv(A)*F0-inv(A)*B*Uc;
F1.show()