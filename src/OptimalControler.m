classdef OptimalControler < handle
    
    properties (SetAccess = protected)
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

        Xmpi; % Maximum Positively Invariant set.
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
            
            obj.construct_costfunction();
            obj.construct_eq_constraint();
            obj.construct_ineq_constraint(Xc, Uc);
            obj.compute_MPIset();
        end

        function reconstruct_ineq_constraint(obj, Xc, Uc)
            % naughty.. must be removed someday
            obj.construct_ineq_constraint(Xc, Uc)
        end

        function remove_initial_eq_constraint(obj)
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
    methods (Access = protected)
        
        function construct_costfunction(obj)
            % compute H
            Q_block = [];
            R_block = [];
            for itr=1:obj.N
                Q_block = blkdiag(Q_block, obj.sys.Q);
                R_block = blkdiag(R_block, obj.sys.R);
            end
            obj.H = blkdiag(Q_block, obj.sys.P, R_block);
        end
        
        function construct_eq_constraint(obj)
            % compute C_eq1 and C_eq2
            A_block = [];
            B_block = [];
            for itr=1:obj.N
                A_block = blkdiag(A_block, obj.sys.A);
                B_block = blkdiag(B_block, obj.sys.B);
            end            
            C_dyn = [zeros(obj.sys.nx, obj.n_opt);
                A_block, zeros(obj.sys.nx*obj.N, obj.sys.nx), B_block]; %Note: [x(0)...x(N)]^T = C_dyn*[x(0)...x(N), u(0)...u(N-1)] + C_eq2
            obj.C_eq1 = eye(obj.sys.nx*(obj.N+1), obj.n_opt) - C_dyn;
            obj.C_eq2 = @(x_init) [x_init; zeros(size(obj.C_eq1, 1)-obj.sys.nx, 1)]; 
        end
       
        function construct_ineq_constraint(obj, Xc, Uc)
            % compute C_neq
            [F, G, obj.nc] = obj.convert_Poly2Mat(Xc, Uc);
            
            F_block = [];
            G_block = [];
            for itr = 1:obj.N
                G_block = blkdiag(G_block, G);
            end
            for itr = 1:obj.N+1
                F_block = blkdiag(F_block, F);
            end
            obj.C_neq1 = [F_block, [G_block; zeros(obj.nc, obj.sys.nu*obj.N)]];
            obj.nc_total = size(obj.C_neq1, 1);
            obj.C_neq2 = ones(obj.nc_total, 1);
        end
        
        function compute_MPIset(obj)
            % MPIset is computed only once in the constructor;
            [F, G, obj.nc] = obj.convert_Poly2Mat(obj.Xc, obj.Uc);
            Fpi = @(i) (F+G*obj.sys.K)*obj.sys.Ak^i;
            Xpi = @(i) Polyhedron(Fpi(i), ones(size(Fpi(i), 1), 1));
            obj.Xmpi = Xpi(0);
            i= 0;
            while(1) % 
                i = i + 1;
                Xmpi_tmp = and(obj.Xmpi, Xpi(i));
                if Xmpi_tmp == obj.Xmpi
                    break;
                else
                    obj.Xmpi = Xmpi_tmp;
                end
            end
        end
        
    end
    
    methods (Access = protected)
        
        function add_ineq_constraint(obj, Xadd, k_add)
            % add a new constraint at time step k 
            if Xadd.contains(zeros(2, 1)) % If Xadd contains the origin, the contraint can be expressed as C1*x<=1
                [F_add, ~, nc_add] = obj.convert_Poly2Mat(Xadd, Polyhedron());
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
        
        function [F, G, nc] = convert_Poly2Mat(obj, X, U)
            % Constraints set X and U in Polyhedron form -> F, G matrix form
            %F; G; % constraints for state and input: Fx+Gu<=1, where 1 is a voctor
            poly2ineq = @(poly) poly.A./repmat(poly.b, 1, size(poly.A, 2));
            F_tmp = poly2ineq(X);
            G_tmp = poly2ineq(U);
            if numel(F_tmp)==0
                F_tmp = zeros(0, obj.sys.nx);
            end
            if numel(G_tmp)==0
                G_tmp = zeros(0, obj.sys.nu);
            end
            F = [F_tmp; zeros(size(G_tmp, 1), obj.sys.nx)];
            G = [zeros(size(F_tmp, 1), obj.sys.nu); G_tmp];
            nc = size(F, 1);
        end
        

        
    end
end

