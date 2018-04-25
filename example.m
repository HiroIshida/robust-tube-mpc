clear all;
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

tmpc = TubeModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N, x_init);
tmpc.simulation();
tmpc.show_result();
%print(gcf,'-djpeg','-r300','sample2.jpg')
