classdef DinvModelPredictiveControl < ModelPredictiveControl
    properties (SetAccess = public)
        Z
        Xc_robust;
        Uc_robust;
        Xmpi_robust;
    end
    
    methods (Access = public)
        function obj = DinvModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N)
            obj@ModelPredictiveControl(A, B, Q, R, Xc, Uc, W, N);
            alpha = 1.1;
            obj.Z = (obj.W+obj.Ak*obj.W+obj.Ak^2*obj.W+obj.Ak^3*obj.W)*alpha;
            obj.Xc_robust = obj.Xc - obj.Z;
            obj.Uc_robust = obj.Uc - obj.K*obj.Z;
            obj.Xmpi_robust = obj.Xmpi - obj.Z;
            obj.construct_ineq_constraint(obj.Xc_robust, obj.Uc_robust); % re-construction of inequality constraint
            obj.add_terminal_constraint(obj.Xmpi_robust);
        end
        
        function [] = simulation(obj, Tsimu, varargin)
            if numel(varargin) == 1
                obj.init(varargin{1})
            elseif obj.flag_init ==0
                error('Error: Please specify the initial state, otherwise initialize object by init()')
            end
            
            x = obj.x_init;
            obj.x_seq_real = x;
            obj.u_seq_real = [];
            obj.time = 1;
            
            for i=1:Tsimu
                u = obj.u_seq_nominal_init(:, obj.time) + obj.K*(x-obj.x_seq_nominal_init(:, obj.time));
                x = obj.propagate(x, u);
                obj.x_seq_real = [obj.x_seq_real, x];
                obj.u_seq_real = [obj.u_seq_real, u];
                obj.time = obj.time + 1;
            end
            clf;
            obj.show_convex(obj.Xc, 'm');
            obj.show_convex(obj.Xc_robust, 'r');
            obj.show_convex(obj.Xmpi, [0.2, 0., 0.6]);
            obj.show_convex(obj.Xmpi_robust, [0.9, 0.9, 0.9]);
            obj.show_trajectory(obj.x_seq_nominal_init, 'gs-');
            for i=1:Tsimu
                obj.show_convex(obj.x_seq_nominal_init(:, i)+obj.Z, 'g', 'FaceAlpha', .2);
                
            end
            obj.show_trajectory(obj.x_seq_real, 'b*-');
        end
        
    end
end