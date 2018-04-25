classdef mpc<handle
    properties (Access = public)
        A; B; % dynamics x[k+1] = A*x[k]+B*u[k]
        Q; R; % quadratic stage cost for LQR
        Xc; Uc; % constraints set for statespace and input space
        F; G; % constraints for state and input: Fx+Gu<=1, where 1 is voctor
        K; % LQR feedback coefficient vector: u=Kx
        P; % optimal cost function of LQR is x'*P*x
        Ak; % S.T.M of closed-roop system with LQR feedback
        nx; % dim. of state space
        nu; % dim. of input
        nc; % dim. of constraint of statespace and input, or dim of Fx+Gu<=1
        n_opt; % dim. of optimization parameter
        N; % prediction horizon
        
        W_real; % real disturbance set (REAL!!)
        w_real_min; % lower bound of disturbance (dim = nx)
        w_real_max; % upper bound of disturbnace (dim = nx)
        
        xc_min; xc_max; % min and max of constraint space 
        uc_min; uc_max; %  min and max of input space
        
        H; % quadratic const V to be minimized here will be expressed as V = s'*H*s, where s:=[x(0), .., x(n-1), x(n), u(0)....u(n-1)].
        C_eq1; C_eq2; % equality constraints are defined as C_eq1*s =C_eq2; C_eq2 is a function object 
        C_neq; % inequality constraints are defined as C_neq*s<=1 (1 is a vector)
    end
    
    properties (Access = public)
        x_current;
        
        x_seq_nominal_init; 
        u_seq_nominal_init
        
        x_seq_nominal;
        u_seq_nominal;
        
        x_seq_real;
        u_seq_real;
    end
    
    properties (Access = protected)
        flag_init = 0;
        flag_init_x = 0;
        flag_dist = 0;
        
        cut_axis1 = 1;
        cut_axis2 = 2;
    end
    
    methods (Access = public)
        %% constructor
        function obj = mpc(A_init, B_init, Q_init, R_init, Xc_init, Uc_init, N_init)
            % init basic parameter
            ppty_list = {'A', 'B', 'Q', 'R', 'Xc', 'Uc', 'N'};
            ppty_list_init = {A_init, B_init, Q_init, R_init, Xc_init, Uc_init, N_init};
            for idx = 1:numel(ppty_list) % initilize
                obj.(ppty_list{idx}) = ppty_list_init{idx};
            end
            
            % init LQR 
            [K_tmp, obj.P] = dlqr(obj.A, obj.B, obj.Q, obj.R);
            obj.K = -K_tmp;
            obj.Ak = (obj.A+obj.B*obj.K);
            
            % dimention of the system
            obj.nx = size(obj.A)*[1; 0];
            obj.nu = size(obj.B)*[0; 1];
            obj.nc = size(obj.Xc.A)*[1; 0]+size(obj.Uc.A)*[1; 0];
            obj.n_opt = obj.nx*(obj.N+1)+obj.nu*obj.N; 
            
            % constraints in an affine form
            F_tmp = obj.Xc.A./repmat(obj.Xc.b, 1, obj.nx);
            G_tmp = obj.Uc.A./repmat(obj.Uc.b, 1, obj.nu);
            obj.F = [F_tmp;
                zeros(size(obj.Uc.A)*[1; 0], obj.nx)];
            obj.G = [zeros(size(obj.Xc.A)*[1; 0], obj.nu);
                G_tmp];
            
            % compute H
            Q_block = [];
            R_block = [];
            for itr=1:obj.N
                Q_block = blkdiag(Q_block, obj.Q);
                R_block = blkdiag(R_block, obj.R);
            end
            obj.H = blkdiag(Q_block, obj.P, R_block);
            
            % compute C_eq1 and C_eq2
            A_block = [];
            B_block = [];
            for itr=1:obj.N
                A_block = blkdiag(A_block, obj.A);
                B_block = blkdiag(B_block, obj.B);
            end            
            C_dyn = [zeros(obj.nx, obj.n_opt);
                A_block, zeros(obj.nx*obj.N, obj.nx), B_block]; %Note: [x(0)...x(N)]^T = C_dyn*[x(0)...x(N), u(0)...u(N-1)] + C_eq2
            obj.C_eq1 = eye(obj.nx*(obj.N+1), obj.n_opt) - C_dyn;
            obj.C_eq2 = @(x_init) [x_init; zeros(size(obj.C_eq1)*[1; 0]-obj.nx, 1)]; 
            
            % compute C_neq
            F_block = [];
            G_block = [];
            for itr = 1:obj.N
                G_block = blkdiag(G_block, obj.G);
            end
            for itr = 1:obj.N+1
                F_block = blkdiag(F_block, obj.F);
            end
            obj.C_neq = [F_block, [G_block; zeros(obj.nc, obj.nu*obj.N)]];
            
            % calcurate the possible min and max of the statespace and input space
            %obj.cvx_set_constr = Polyhedron('A', [obj.F, obj.G], 'b', ones(obj.nc, 1)); 
            
            obj.xc_min = min(obj.Xc.V, [], 1);
            obj.xc_max = max(obj.Xc.V, [], 1);
            obj.uc_min = min(obj.Uc.V, [], 1);
            obj.uc_max = max(obj.Uc.V, [], 1);
        end
        
        function construct_basic_constraints(obj, Xc, Uc)
            % make constraint in the form of Fx+Gu<=1
        end
        function add_constraint(obj, Xc, Uc)
            % add constraint 
        end
        
        
        function disturbance(obj, W_init)
            obj.W_real = W_init;
            obj.w_real_min = min(W_init.V, [], 1)';
            obj.w_real_max = max(W_init.V, [], 1)';
            obj.flag_dist = 1;
        end
        
        function init_x(obj, x)
            obj.x_current = x;
            obj.flag_init_x = 1;
            obj.x_seq_real = x;
            obj.solve(); % to obtain a initial nominal trajectory
        end
        
        %% basic solver
        function input = solve(obj)
            x = obj.x_current;            
            quad_prog_solved = 0;
            C_neq_current = obj.C_neq;
            count_itr = 0;
            while(quad_prog_solved ~= 1)
                options = optimoptions('quadprog', 'Display', 'none');
                [s_star, ~, exitflag] = quadprog(obj.H, [], C_neq_current, ones(1, size(C_neq_current)*[1; 0]), obj.C_eq1, obj.C_eq2(x), [], [], [], options);
                quad_prog_solved = (exitflag==1);
                C_neq_current = C_neq_current*0.999; % constraints relaxation 
                count_itr = count_itr + 1;
                if (count_itr>10)
                    error('Not feasible');
                end
            end
            
            obj.x_seq_nominal = reshape(s_star(1:obj.nx*(obj.N+1), 1), obj.nx, obj.N+1);
            obj.u_seq_nominal = reshape(s_star(obj.nx*(obj.N+1)+1:obj.n_opt), obj.nu, obj.N);
            input = obj.u_seq_nominal(1,1);
            
            if obj.flag_init == 0
                obj.x_seq_nominal_init = obj.x_seq_nominal;
                obj.u_seq_nominal_init = obj.u_seq_nominal;
                obj.flag_init = 1;
            end
        end
        
        function [] = propagate(obj)
            input = solve(obj);
            if obj.flag_dist == 1
                w = obj.w_real_min+rand(2, 1).*(obj.w_real_max-obj.w_real_min);
            else
                w = 0;
            end
            obj.x_current = obj.A*obj.x_current+obj.B*input+w;
            obj.x_seq_real = [obj.x_seq_real, obj.x_current];
            obj.u_seq_real = [obj.u_seq_real, input];
        end
        
        %% graphics
        % maybe these type of function should be located in a sub classs 
        function [] = cut_surface(obj, axis1, axis2)
            if or(axis1>obj.nx, axis2>obj.nx)
                error('NG');
            else
            obj.cut_axis1 = axis1;
            obj.cut_axis2 = axis2;
            end
        end
        
        function show_constraint(obj, set, color_fill, varargin)
            vtx_constr = mpc.ptope2vtx(set, obj.cut_axis1, obj.cut_axis2);
            if numel(varargin)==0
                fill(vtx_constr(:,1), vtx_constr(:,2), color_fill);
            else
                fill(vtx_constr(:,1), vtx_constr(:,2), color_fill, varargin{1}, varargin{2});
            end
            hold on;
        end

        function [] = show_Xc(obj)
            obj.show_constraint(obj.Xc, 'm');
        end
        
        function [] = show_init_nominal_traj(obj, varargin)
            % line type
            if numel(varargin) == 1
                line_type = varargin{1};
            else
                line_type = 'gs-';
            end
            plot(obj.x_seq_nominal_init(obj.cut_axis1, :), obj.x_seq_nominal_init(obj.cut_axis2, :), line_type);
            hold on;
        end
        
        function [] = show_real_traj(obj, varargin)
            if numel(varargin) == 1
                line_type = varargin{1};
            else
                line_type = 'bo-';
            end
            plot(obj.x_seq_real(1, :), obj.x_seq_real(2, :), line_type);
            hold on;
        end
        
        function [] = show_auto_limit(obj)
            xlim([obj.xc_min(obj.cut_axis1)-0.5, obj.xc_max(obj.cut_axis1)+0.5]);
            ylim([obj.xc_min(obj.cut_axis2)-0.5, obj.xc_max(obj.cut_axis2)+0.5]);
        end

    end
    
    methods (Static)
                    
        function vtx = ptope2vtx(P, varargin)
            vtx_tmp = P.V;
            if numel(varargin)==0
                cut_axis1 = 1;
                cut_axis2 = 2;
            else
                cut_axis1 = varargin{1};
                cut_axis2 = varargin{2};
            end
            idx_proper = convhull(round(vtx_tmp(:, cut_axis1), 5), round(vtx_tmp(:, cut_axis2), 5));
            cut_surface = [cut_axis1, cut_axis2];
            vtx = vtx_tmp(idx_proper, cut_surface);
        end
    end
end