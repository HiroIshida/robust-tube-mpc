classdef OptimalControler < handle
    
    properties (SetAccess = private)
        sys; %system
        Xc; Uc; % constraints set for statespace and input space
        x_min; x_max; % lower and upper bound of Xc
        nc; % number of contraint of Xc + Uc
        N; % prediction horizon
        Ak; % S.T.M of closed-roop system with LQR feedback
        n_opt; % dim. of optimization parameter
        nc_total; % number of all ineq. constraints 
        H; % quadratic const V to be minimized here will be expressed as V = s'*H*s, where s:=[x(0), .., x(n-1), x(n), u(0)....u(n-1)].
        C_eq1; C_eq2; % equality constraints are defined as C_eq1*s =C_eq2; C_eq2 is a function object 
        C_neq1; C_neq2; % inequality constraints are defined as C_neq*s<=1 (1 is a vector)
        constraint_manager
    end
    
    %% Public Methods
    methods (Access = public)
        
        function obj = OptimalControler(sys, Xc, Uc, N)
            obj.sys = sys
            obj.Xc = Xc;
            obj.x_min = min(Xc.V, [], 1)';
            obj.x_max = max(Xc.V, [], 2)';
            obj.Uc = Uc;
            obj.N = N;
            
            obj.n_opt = obj.sys.nx*(obj.N+1)+obj.sys.nu*obj.N;
            obj.constraint_manager = ConstraintManager()
            
            obj.H = obj.construct_costfunction();
            [obj.C_eq1, obj.C_eq2] = obj.construct_eq_constraint();
            [obj.C_neq1, obj.C_neq2] = obj.construct_ineq_constraint(Xc, Uc);
        end

        function remove_initial_eq_constraint(obj)
            % In tube MPC, the initial state of the nominal trjectory must not 
            % be the actual initial state (see section 4 of Mayne et al, Robust model predictive control of constrained linear systems with bounded disturbances, 2005)
            % this function will be used in tube model predictive control
            obj.C_eq1 = obj.C_eq1(obj.sys.nx+1:size(obj.C_eq1, 1), 1:size(obj.C_eq1, 2));
            obj.C_eq2 = @(x) zeros(size(obj.C_eq1, 1), 1);
        end
        
        function add_terminal_constraint(obj, Xadd)
            add_ineq_constraint(obj, Xadd, obj.N+1);
        end

        function add_initial_constraint(obj, Xadd)
            add_ineq_constraint(obj, Xadd, 1);
        end
        
        function [x_seq, u_seq] = solve(obj, x_init)
            quadprog_solved = 0;
            C_neq1_relaxed = obj.C_neq1;
            options = optimoptions('quadprog', 'Display', 'none');
            itr = 0;
            while(quadprog_solved ~=1 )
                [var_optim, ~, exitflag] = quadprog(obj.H, [], C_neq1_relaxed, obj.C_neq2, obj.C_eq1, obj.C_eq2(x_init), [], [], [], options);
                
                quadprog_solved = (exitflag==1);
                C_neq1_relaxed = C_neq1_relaxed*0.999;
                itr = itr + 1;
                if (itr>10)
                    error('Not feasible');
                end
            end
            x_seq = reshape(var_optim(1:obj.sys.nx*(obj.N+1)), obj.sys.nx, obj.N+1);
            u_seq = reshape(var_optim(obj.sys.nx*(obj.N+1)+1:obj.n_opt), obj.sys.nu, obj.N);
            
        end
        
    end
    
    %% Methods Used in Constoructor
    methods (Access = private)
        
        function H = construct_costfunction(obj)
            % compute H
            Q_block = [];
            R_block = [];
            for itr=1:obj.N
                Q_block = blkdiag(Q_block, obj.sys.Q);
                R_block = blkdiag(R_block, obj.sys.R);
            end
            H = blkdiag(Q_block, obj.sys.P, R_block);
        end
        
        function [C_eq1, C_eq2] = construct_eq_constraint(obj)
            % compute C_eq1 and C_eq2
            A_block = [];
            B_block = [];
            for itr=1:obj.N
                A_block = blkdiag(A_block, obj.sys.A);
                B_block = blkdiag(B_block, obj.sys.B);
            end            
            C_dyn = [zeros(obj.sys.nx, obj.n_opt);
                A_block, zeros(obj.sys.nx*obj.N, obj.sys.nx), B_block]; %Note: [x(0)...x(N)]^T = C_dyn*[x(0)...x(N), u(0)...u(N-1)] + C_eq2
            C_eq1 = eye(obj.sys.nx*(obj.N+1), obj.n_opt) - C_dyn;
            C_eq2 = @(x_init) [x_init; zeros(size(obj.C_eq1, 1)-obj.sys.nx, 1)]; 
        end
       
        function [C_neq1, C_neq2] = construct_ineq_constraint(obj, Xc, Uc)
            % compute C_neq
            [F, G, obj.nc] = convert_Poly2Mat(Xc, Uc);
            
            F_block = [];
            G_block = [];
            for itr = 1:obj.N
                G_block = blkdiag(G_block, G);
            end
            for itr = 1:obj.N+1
                F_block = blkdiag(F_block, F);
            end
            C_neq1 = [F_block, [G_block; zeros(obj.nc, obj.sys.nu*obj.N)]];
            obj.nc_total = size(C_neq1, 1);
            C_neq2 = ones(obj.nc_total, 1);
        end
        
    end
    
    methods (Access = private)
        
        function add_ineq_constraint(obj, Xadd, k_add)
            % add a new constraint at time step k 
            if Xadd.contains(zeros(2, 1)) % If Xadd contains the origin, the contraint can be expressed as C1*x<=1
                [F_add, ~, nc_add] = convert_Poly2Mat(Xadd, Polyhedron());
                obj.C_neq2 = [obj.C_neq2; ones(nc_add, 1)];
                
            else % in other cases, expressed in a general affine form C1*x<=C2
                F_add = Xadd.A;
                nc_add = size(F_add, 1);
                obj.C_neq2 = [obj.C_neq2; Xadd.b];
            end
            
            block_add = zeros(nc_add, obj.n_opt);
            block_add(:, (k_add-1)*obj.sys.nx+1:k_add*obj.sys.nx) = F_add;
            
            obj.C_neq1 = [obj.C_neq1;
                block_add];
            obj.nc = obj.nc + nc_add;
        end
        
    end
end

