addpath('../src/')
addpath('../src/utils/')

% specify your own discrete linear system
A = [1 1; 0 1];
B = [0.5; 1]; 
Q = diag([1, 1]);
R = 0.1;

% construct a convex set of system noise (2dim here)
W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
W = Polyhedron(W_vertex);

% construct disturbance Linear system
% note that disturbance invariant set Z is computed and stored as member variable in the constructor.
disturbance_system = DisturbanceLinearSystem(A, B, Q, R, W); 

% you can see that with any disturbance bounded by W, the state is guaranteed to inside Z
x = zeros(2); % initial state 
for i = 1:50
    u = disturbance_system.K * (x - 0);
    x = disturbance_system.propagate(x, u); % disturbance is considered inside the method
    clf;
    Graphics.show_convex(disturbance_system.Z, 'g', 'FaceAlpha', .3); % show Z
    scatter(x(1, :), x(2, :)); % show particle
    pause(0.01);
end
