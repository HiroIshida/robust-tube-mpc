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

% compute disturvance invariant set Z.
Z = mysys.compute_distinv_set(W, 3, 1.05);

% set boundary of system noise which corresponds to W.
w_max = [0.15; 0.15];
w_min = [-0.15; -0.15];

% propagate particles many times following the discrete stochastic dynamics,
% and we will see that particles never go outside of distervance invariant set Z.
Nptcl = 10000;
x = zeros(2, Nptcl); % particles 
for i = 1:100
    sys_noise = rand(2, Nptcl).*repmat(w_max - w_min, 1, Nptcl) + repmat(w_min, 1, Nptcl);
    x = mysys.Ak*x + sys_noise;
    clf;
    Graphics.show_convex(Z, 'g', 'FaceAlpha', .3); % show Z
    scatter(x(1, :), x(2, :)); % show particles
    pause(0.01)
end
