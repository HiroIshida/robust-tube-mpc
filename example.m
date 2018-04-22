A = [1 1; 0 1]; B = 0.5*[1; 2];
Q = ones(2); R = 0.1; 
F = [-0.1, 0;
    0.1, 0;
    0, -0.5;
    0, 0.5;
    0, 0;
    0, 0];
G = [0; 0; 0; 0; -1; 1];
v = [0.5, 0.5; 0.5, -0.5; -0.5, -0.5; -0.5, 0.5]*0.5;
W = Polyhedron('V', v);
N = 10;
Xc = Polyhedron(F(1:4, :), ones(4, 1));
Uc = Polyhedron(G(5:6, :), ones(2, 1));
mp = tube_mpc(A, B, Q, R, Xc, Uc, N, W);
x_init = [8.6; 1];
mp.init_x(x_init);
clf
for i=1:100
    mp.propagate();
end
mp.show_Xc();
mp.show_Xc_robust();
mp.show_Xter();
mp.show_Xter_robust();
mp.show_tube();

legend('Xc (state space constraint)', 'Xc - Z (robust space constraint)', 'Xter (terminal constraint)', 'Xter - Z (robust terminal constraint)', 'tube');
mp.show_init_nominal_traj();
mp.show_real_traj();
mp.show_auto_limit();
xlabel('x1');
ylabel('x2');
print(gcf,'-djpeg','-r300','sample.jpg')
