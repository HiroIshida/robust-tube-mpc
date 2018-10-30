classdef ModelPredictiveControl < GraphicsBasis
    
    properties (SetAccess = protected)
        sys % linear sys
        optimal_controler; % optimal contol solver object
        Xc
        w_min; w_max; % lower and upper bound of system noise
        % each vector has the same dim as that of system

        
        x_init; % initial state vector
        x_seq_nominal_init; % nominal optimal trajectory computed from x_init
        u_seq_nominal_init; % nominal optimal input-sequence computed from x_init
        
        Tsimu; % simulation time span

        x_seq_real; % accumulation of the real x's sequence
        u_seq_real; % accumulation of the real u's sequence
        
        time; % current time step
    end
    
    properties (Access = protected)
        flag_init = 0;
    end
    
    methods (Access = public)
        
        function obj = ModelPredictiveControl(sys, Xc, Uc, N, varargin)
            obj@GraphicsBasis();
            obj.sys = sys;
            obj.Xc = Xc
            obj.optimal_controler = OptimalControler(sys, Xc, Uc, N);
            if numel(varargin)==2 % wanna write in Julia... 
                obj.w_min = varargin{1}
                obj.w_max = varargin{2}

            else % no arguments sets no system noise
                obj.w_min = zeros(2, 1)
                obj.w_max = zeros(2, 1)
            end
        end
        
        function [] = init(obj, x_init)
            obj.flag_init = 1;
            obj.x_init = x_init;
            [x_seq, u_seq] = obj.optimal_controler.solve(x_init);
            obj.x_seq_nominal_init = x_seq;
            obj.u_seq_nominal_init = u_seq;
        end
        
        function [] = simulation(obj, Tsimu, x_init)
            x_init
            obj.init(x_init)
            
            x = obj.x_init;
            obj.x_seq_real = x;
            obj.u_seq_real = [];
            obj.time = 1;
            propagate = @(x, u, w) obj.sys.A*x+obj.sys.B*u + w;
            
            for i=1:Tsimu
                u = obj.compute_OptimalInput(x);
                w = randn(2, 1).*(obj.w_max - obj.w_min)+obj.w_min;
                x = propagate(x, u, w);
                obj.x_seq_real = [obj.x_seq_real, x];
                obj.u_seq_real = [obj.u_seq_real, u];
                obj.time = obj.time + 1;
            end
            clf;
            obj.show_convex(obj.Xc, 'r');
            obj.show_trajectory(obj.x_seq_nominal_init, 'gs-');
            obj.show_trajectory(obj.x_seq_real, 'b*-');
        end
        
    end
    
    methods (Access = protected)
        
        function input = compute_OptimalInput(obj, x)
            [~, u_seq] = obj.optimal_controler.solve(x);
            input = u_seq(:, 1);
        end
    end
    
end
