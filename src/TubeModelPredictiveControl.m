classdef TubeModelPredictiveControl < ModelPredictiveControl
    
    properties (SetAccess = public)
        Z % disturbance invariant set
        Xc_robust; % Xc-Z (Pontryagin diff.)
        Uc_robust; % Uc-K*Z (Pontryagin diff.)
        Xmpi_robust; % Xmpi-Z (Pontryagin diff.)
        idx_enter; % outside or not of Xmpi_robust (if inside, just the LQR-control law is applied)
    end
    
    methods (Access = public)
        function obj = TubeModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N, x_init)
            obj@ModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N);
            
            %----------approximation of d-inv set--------%
            alpha = 1.1;
            obj.Z = (obj.W+obj.Ak*obj.W+obj.Ak^2*obj.W+obj.Ak^3*obj.W)*alpha;
            
            obj.Xc_robust = obj.Xc - obj.Z;
            obj.Uc_robust = obj.Uc - obj.K*obj.Z;
            obj.Xmpi_robust = obj.Xmpi - obj.Z;
            obj.construct_ineq_constraint(obj.Xc_robust, obj.Uc_robust); % re-construction of inequality constraint
            obj.add_terminal_constraint(obj.Xmpi_robust);
            
           %-------------buggy-start-------------------------%
            obj.x_init = x_init;
            obj.C_eq1 = obj.C_eq1(obj.nx+1:size(obj.C_eq1)*[1; 0], 1:size(obj.C_eq1)*[0; 1]);
            obj.C_eq2 = @(x) zeros(size(obj.C_eq1)*[1;0], 1);
            Xinit = x_init+obj.Z;
            obj.add_initial_constraint(Xinit);
            %obj.show_convex(x_init+obj.Z, 'r')
            %-------------buggy-end-------------------------%
            obj.init();
        end
        
        function [] = init(obj) % override
            obj.flag_init = 1;
            [x_seq, u_seq] = obj.solve_OptimalControl(obj.x_init);
            obj.x_seq_nominal_init = x_seq;
            obj.u_seq_nominal_init = u_seq;
            
            obj.idx_enter = obj.N;
            for k=1:obj.N
                if obj.Xmpi_robust.contains(x_seq(:, k))
                    obj.idx_enter = k;
                    break;
                end
            end
        end
        
        function [] = simulation(obj, varargin)
            
            if numel(varargin) == 1
                obj.init(varargin{1})
            elseif obj.flag_init ==0
                error('Error: Please specify the initial state, otherwise initialize object by init()')
            end
            
            x = obj.x_init;
            obj.x_seq_real = x;
            obj.u_seq_real = [];
            obj.time = 1;
            for i=1:obj.N
                %{
                if i>=obj.idx_enter 
                    u = obj.K*x; % LQR
                else
                    u = obj.u_seq_nominal_init(:, obj.time) + obj.K*(x-obj.x_seq_nominal_init(:, obj.time));
                end
                %}
                u = obj.u_seq_nominal_init(:, obj.time) + obj.K*(x-obj.x_seq_nominal_init(:, obj.time));
                x = obj.propagate(x, u);
                obj.x_seq_real = [obj.x_seq_real, x];
                obj.u_seq_real = [obj.u_seq_real, u];
                obj.time = obj.time + 1;
            end
            
        end
        
        function [] = show_result(obj)
            clf;
            obj.show_convex_timeslice(obj.Xc, -0.03, 'm');
            obj.show_convex_timeslice(obj.Xc_robust, -0.02, 'r');
            obj.show_convex_timeslice(obj.Xmpi, -0.01, [0.2, 0.2, 0.2]*1.5);
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