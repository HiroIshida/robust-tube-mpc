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
        function obj = TubeModelPredictiveControl(sys, Xc, Uc, W, N)
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
            x_seq_real = x;
            u_seq_real = [];
            propagate = @(x, u, w) obj.sys.A*x+obj.sys.B*u + w;
            for i=1:Tsimu
                u = u_nominal_seq(:, i) + obj.sys.K*(x-x_nominal_seq(:, i));
                w = [0; 0];
                x = propagate(x, u, w);
                clf;
                graph.show_convex(obj.Xc, 'm');
                graph.show_convex(obj.Xc_robust, 'r');
                graph.show_convex(obj.Xmpi_robust, [0.5, 0.5, 0.5]); % gray
                for j=1:obj.N+1
                    graph.show_convex(x_nominal_seq(:, j)+obj.Z, 'g', 'FaceAlpha', 0.3);
                end
                graph.show_trajectory(x_nominal_seq, 'gs-');
                graph.show_trajectory(x, 'b*-');
                pause(0.2)
            end
            
        end
        
        function [] = show_result(obj)
            clf;
            obj.show_convex_timeslice(obj.Xc, -0.03, 'm');
            obj.show_convex_timeslice(obj.Xc_robust, -0.02, 'r');
            obj.show_convex_timeslice(obj.optcon.Xmpi, -0.01, [0.2, 0.2, 0.2]*1.5);
            obj.show_convex_timeslice(obj.Xmpi_robust, -0.005, [0.5, 0.5, 0.5]);
            obj.show_convex_timeslice(obj.x_seq_nominal_init(:, 1)+obj.Z, obj.N, 'g', 'FaceAlpha', .3);
            
            obj.show_trajectory_timeslice(obj.x_seq_nominal_init, 'gs-', 'LineWidth', 1.2);
            obj.show_trajectory_timeslice(obj.x_seq_real, 'b*-', 'LineWidth', 1.2);
            
            leg = legend('$X_c$', '$X_c\ominus Z$', '$X_f (= X_{MPI})$', '$X_f\ominus Z$', 'Tube', 'Nominal', 'Real');
            set(leg, 'Interpreter', 'latex')
            
            for i=2:obj.N+1 % show remaining tubes.
                    obj.show_convex_timeslice(obj.x_seq_nominal_init(:, i)+obj.Z, obj.N-i+1, 'g', 'FaceAlpha', .3);
            end
            
            xlabel('x1');
            ylabel('x2');
            zlabel('time (minus)');
            xlim([obj.x_min(obj.axis1), obj.x_max(obj.axis1)]);
            ylim([obj.x_min(obj.axis2), obj.x_max(obj.axis2)]);
            zlim([-0.05, obj.N]);
            set( gca, 'FontSize',12); 
            %view([0, 90]);
            view([10, 10])
            grid on;
            
      
        end
        
    end
end
