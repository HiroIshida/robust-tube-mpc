% tutorial for understanding tube-MPC
progn = @(varargin) varargin{end};
idx_proper = @(P) convhull(P.V(:,1), P.V(:,2));
plot_conv = @(P, varargin) progn(...
    plot(P.V(idx_proper(P), 1), P.V(idx_proper(P), 2), varargin{1}, varargin{2}, varargin{3}), ...
    evalc('hold on'));
fill_conv = @(P, varargin) progn(...
    fill(P.V(idx_proper(P), 1), P.V(idx_proper(P), 2), varargin{1}, varargin{2}, varargin{3}), ...
    evalc('hold on'));
plot_trajectory = @(sequence, varargin) progn(...
    plot(sequence(1, :), sequence(2, :), varargin{1}, varargin{2}, varargin{3}),...
evalc('hold on'));
    

% model and cost definition
A = [1 1; 0 1]; B = [0.5; 1]; 
Q = diag([1, 1]); R = 0.1;
Xc_vertex = [5, -2; 5 2; -10 2; -10 -2];
Uc_vertex = [1; -1];
Xc = Polyhedron(Xc_vertex);
Uc = Polyhedron(Uc_vertex);


W_vertex = [0.15, 0.15; 0.15, -0.15; -0.15, -0.15; -0.15, 0.15];
W = Polyhedron(W_vertex);
Ak = (A + B*mp.K); % autonomous dynamics
Z = (W+Ak*W+Ak^2*W+Ak^3*W)*1.05;

N = 7;
x_init = [-8; 0];
%x_init = [-6; -1];
mp = OptimalControlBasis(A, B, Q, R, Xc-Z, Uc-mp.K*Z, N);
mp.add_terminal_constraint( mp.Xmpi - Z);


% ÉçÉoÉXÉg
[x_nominal, u_nominal] = mp.solve_OptimalControl(x_init);
x = x_init;
Tsimu = 100;
for t=1:Tsimu
    if t<=N
        u = u_nominal(:, t)+mp.K*(x-x_nominal(:, t));
    else
        u = mp.K*x;
    end
   w = rand(2, 1)*0.3-[0.15; 0.15];
    x = A*x + B*u+w;
    
    clf;
    plot_conv(Xc, 'm', 'Linewidth', 3);
    plot_conv(Xc-Z, 'r', 'LineWidth', 3);
    plot_conv(mp.Xmpi, 'k', 'LineWidth', 3);
    plot_conv(mp.Xmpi-Z, 'k', 'LineWidth', 3);
    plot_trajectory(x_nominal, 'go-', 'LineWidth', 2);
    for i=1:N+1
        fill_conv(Z+x_nominal(:, i), 'g', 'FaceAlpha', .3);
    end
    
    
    plot_trajectory(x, 'b*', 'LineWidth', 2);
    grid on;
    pause(0.2);
end
