clear all
% tutorial for understanding tube-MPC

% graphics
%{
progn = @(varargin) varargin{end};
idx_proper = @(P) convhull(P.V(:,1), P.V(:,2));
plot_conv = @(P, varargin) progn(...
    plot(P.V(idx_proper(P), 1), P.V(idx_proper(P), 2), varargin{1}, varargin{2}, varargin{3}), ...
    evalc('hold on'));
plot_trajectory = @(sequence, varargin) progn(...
    plot(sequence(1, :), sequence(2, :), varargin{1}, varargin{2}, varargin{3}),...
evalc('hold on'));
%}

% define plot_conv
idx_proper = @(P) convhull(P.V(:,1), P.V(:,2));
plot_conv = @(P, varargin) plot(P.V(idx_proper(P), 1), P.V(idx_proper(P), 2), varargin{1}, varargin{2}, varargin{3});

% model and cost definition
A = [1 1; 0 1]; B = [0.5; 1]; 
Q = diag([1, 1]); R = 0.1;
Xc_vertex = [5, -2; 5 2; -10 2; -10 -2];
Uc_vertex = [0.1; -0.1]*10;
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);
N = 10;

% simulation
mp = OptimalControlBasis(A, B, Q, R, Xc, Uc, 10);
x_init = [-10; 1];
[x_nominal, u_nominal] = mp.solve_OptimalControl(x_init);
plot_conv(Xc, 'r', 'Linewidth', 3); hold on;
plot(x_nominal(1, :), x_nominal(2, :), 'go-', 'LineWidth', 2);
grid on;


%{
% simulation 
x_init = [-10; 1];
x = x_init;
[x_nominal, u_nominal] = mp.solve_OptimalControl(x);
for t=1:N
    clf;
    plot_conv(Xc, 'r', 'Linewidth', 3);
    plot_conv(mp.Xmpi, 'k', 'LineWidth', 3);
    plot_trajectory(x_nominal, 'go-', 'LineWidth', 2);
    plot_trajectory(x, 'b*', 'LineWidth', 2);
    pause(0.1);
    u = u_nominal(:, t);
    x = A*x + B*u;
end
%}


%äOóêÇ†ÇË
%{
for t=1:20
    [x_nominal, u_nominal] = mp.solve_OptimalControl(x);
    u = u_nominal(:, 1);
   w = rand(2, 1)*0.3-[0.15; 0.15];
    x = A*x + B*u+w;
    
    clf;
    plot_conv(Xc, 'r', 'Linewidth', 3);
    plot_conv(mp.Xmpi, 'k', 'LineWidth', 3);
    plot_trajectory(x_nominal, 'go-', 'LineWidth', 2);
    plot_trajectory(x, 'b*', 'LineWidth', 2);
    pause(0.5);
end
%}

