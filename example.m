clear all;
%{
A = [1 1; 0 1]; B = [0.5; 1];
Q = ones(2); R = 0.1; 
F = [-0.1, 0;
    0.5, 0;
    0, -0.5;
    0, 0.5;
    0, 0;
    0, 0];
G = [0; 0; 0; 0; -1; 1];
v = [0.5, 0.5; 0.5, -0.5; -0.5, -0.5; -0.5, 0.5]*0.3;
W = Polyhedron('V', v);
N = 10;
Xc = Polyhedron(F(1:4, :), ones(4, 1));
Uc = Polyhedron(G(5:6, :), ones(2, 1));
x_init = [-7; -2];
%}

A = [1 1; 0 1]; B = [0.5; 1]; 
Q = diag([1, 1]); R = 0.1;
Xc_vertex = [2, -2; 2 2; -10 2; -10 -2];
Uc_vertex = [1; -1];
W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
W = Polyhedron(W_vertex);

x_init = [-7; -2]; N = 10;

tmpc = TubeModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N, x_init);
tmpc.simulation();
tmpc.show_result();
%print(gcf,'-djpeg','-r300','sample2.jpg')
