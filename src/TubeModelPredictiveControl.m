classdef TubeModelPredictiveControl
    
    properties (SetAccess = public)
        sys % linear sys
        optcon; % optimal contol solver object
        Xc
        Uc
        Xc_robust; % Xc-Z (Pontryagin diff.)
        %Uc_robust; % Uc-K*Z (Pontryagin diff.)
        W
        Z % disturbance invariant set
        Xmpi_robust; % Xmpi-Z (Pontryagin diff.)
        N
    end
    
    methods (Access = public)
        function obj = TubeModelPredictiveControl(sys, Xc, Uc, W, N, w_min, w_max)
            optcon = OptimalControler(sys, Xc, Uc, N)
            %----------approximation of d-inv set--------%
            alpha = 1.1;
            Z = (W+sys.Ak*W+sys.Ak^2*W+sys.Ak^3*W)*alpha;

            %create robust X and U constraints, and construct solver using X and U
            Xc_robust = Xc - Z;
            Uc_robust = Uc - sys.K*Z;
            optcon.reconstruct_ineq_constraint(Xc_robust, Uc_robust)

            %robustize Xmpi set and set it as a terminal constraint
            Xmpi_robust = optcon.Xmpi - Z;
            optcon.add_terminal_constraint(Xmpi_robust);

            %fill properteis
            obj.sys = sys;
            obj.optcon = optcon
            obj.W = W;
            obj.Xc = Xc;
            obj.Uc = Uc;
            obj.Xc_robust = Xc_robust;
            obj.W = W
            obj.Z = Z
            obj.Xmpi_robust = Xmpi_robust
            obj.N = N
        end
        
        function [] = simulate(obj, Tsimu, x_init)
            graph = Graphics();
            obj.optcon.remove_initial_eq_constraint()
            Xinit = x_init+obj.Z;
            obj.optcon.add_initial_constraint(Xinit);

            [x_nominal_seq, u_nominal_seq] = obj.optcon.solve(x_init);
            
            x = x_init;
            x_real_seq = [x];
            u_real_seq = [];
            propagate = @(x, u, w) obj.sys.A*x+obj.sys.B*u + w;
            % simulation loop
            for i=1:Tsimu
                if i<=obj.N
                    u = u_nominal_seq(:, i) + obj.sys.K*(x-x_nominal_seq(:, i));
                else 
                    u = obj.sys.K*x;
                end
                w = [0; 0];
                x = propagate(x, u, w);
                x_real_seq = [x_real_seq, x];
                u_real_seq = [u_real_seq, u];

                clf; % real time plot
                graph.show_convex(obj.Xc, 'm');
                graph.show_convex(obj.Xc_robust, 'r');
                graph.show_convex(obj.optcon.Xmpi, [0.2, 0.2, 0.2]*1.5);
                graph.show_convex(obj.Xmpi_robust, [0.5, 0.5, 0.5]); % gray
                for j=1:obj.N+1
                    graph.show_convex(x_nominal_seq(:, j)+obj.Z, 'g', 'FaceAlpha', 0.3);
                end
                graph.show_trajectory(x_nominal_seq, 'gs-');
                graph.show_trajectory(x, 'b*-');
                pause(0.2)
            end

            % time slice plot after simulation
            figure(2)
            graph.show_convex_timeslice(obj.Xc, -0.04, 'm');
            graph.show_convex_timeslice(obj.Xc_robust, -0.03, 'r');
            graph.show_convex_timeslice(obj.optcon.Xmpi, -0.02, [0.2, 0.2, 0.2]*1.5);
            graph.show_convex_timeslice(obj.Xmpi_robust, -0.01, [0.5, 0.5, 0.5]);
            graph.show_convex_timeslice(x_nominal_seq(:, 1)+obj.Z, obj.N, 'g', 'FaceAlpha', .3);
            graph.show_trajectory_timeslice(x_nominal_seq, 'gs-', 'LineWidth', 1.2);
            graph.show_trajectory_timeslice(x_real_seq(:, 1:obj.N+1), 'b*-', 'LineWidth', 1.2);
            leg = legend('$X_c$', '$X_c\ominus Z$', '$X_f (= X_{MPI})$', '$X_f\ominus Z$', 'Tube', 'Nominal', 'Real');
            set(leg, 'Interpreter', 'latex')
            
            for i=2:obj.N+1 % show remaining tubes.
                    graph.show_convex_timeslice(x_nominal_seq(:, i)+obj.Z, obj.N-i+1, 'g', 'FaceAlpha', .3);
            end
            
            xlabel('x1');
            ylabel('x2');
            zlabel('time (minus)');
            xlim([obj.optcon.x_min(1), obj.optcon.x_max(1)]);
            ylim([obj.optcon.x_min(2), obj.optcon.x_max(2)]);
            zlim([-0.05, obj.N]);
            set( gca, 'FontSize',12); 
            %view([0, 90]);
            view([10, 10])
            grid on;

        end
        
    end
end
