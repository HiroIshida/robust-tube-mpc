addpath('./src')
addpath('./utils')
A = [1 1; 0 1];
B = [0.5; 1]; 
Q = diag([1, 1]);
R = 0.1;
mysys = LinearSystem(A, B, Q, R);

Xc_vertex = [2, -2; 2 2; -10 2; -10 -2];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);

opt = OptimalControler(mysys, Xc, Uc, 20)


x_init = [-7; -2]

[x_seq, u_seq] = opt.solve(x_init)

%{
W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
W = Polyhedron(W_vertex);

x_init = [-7; -2]; N = 10;

tmpc = TubeModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N, x_init);
tmpc.simulation();
tmpc.show_result();
%print(gcf,'-djpeg','-r300','sample2.jpg')
%}

