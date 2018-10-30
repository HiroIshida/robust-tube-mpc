classdef OptimalControlBasis < handle
    
    properties (SetAccess = protected)
        
        A; B; % dynamics x[k+1] = A*x[k]+B*u[k]
        nx; nu; % dim of state space and input space
        Q; R; % quadratic stage cost for LQR
        Xc; Uc; % constraints set for statespace and input space
        x_min; x_max; % lower and upper bound of Xc

        nc; % number of contraint of Xc + Uc
        N; % prediction horizon

        K; % LQR feedback coefficient vector: u=Kx
        P; % optimal cost function of LQR is x'*P*x
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
        
        function obj = OptimalControlBasis(A, B, Q, R, Xc, Uc, N)
            obj.A = A;
            obj.B = B;
            obj.nx = size(A)*[1; 0];
            obj.nu = size(B)*[0; 1];
            obj.Q = Q;
            obj.R = R;
            obj.Xc = Xc;
            obj.x_min = min(Xc.V, [], 1)';
            obj.x_max = max(Xc.V, [], 2)';
            obj.Uc = Uc;
            obj.N = N;
            
            [K_tmp, obj.P] = dlqr(obj.A, obj.B, obj.Q, obj.R);
            obj.K = -K_tmp;
            obj.Ak = (obj.A+obj.B*obj.K);
            obj.n_opt = obj.nx*(obj.N+1)+obj.nu*obj.N;
            
            obj.construct_costfunction();
            obj.construct_eq_constraint();
            obj.construct_ineq_constraint(Xc, Uc);
            obj.compute_MPIset();
        end
        
        function add_terminal_constraint(obj, Xadd)
            add_ineq_constraint(obj, Xadd, obj.N+1);
        end
        
        function add_initial_constraint(obj, Xadd)
            add_ineq_constraint(obj, Xadd, 1);
        end
        
        function [x_seq, u_seq] = solve_OptimalControl(obj, x_init)
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
            x_seq = reshape(var_optim(1:obj.nx*(obj.N+1)), obj.nx, obj.N+1);
            u_seq = reshape(var_optim(obj.nx*(obj.N+1)+1:obj.n_opt), obj.nu, obj.N);
            
        end
        
    end
    
    %% Methods Used in Constoructor
    methods (Access = protected)
        
        function construct_costfunction(obj)
            % compute H
            Q_block = [];
            R_block = [];
            for itr=1:obj.N
                Q_block = blkdiag(Q_block, obj.Q);
                R_block = blkdiag(R_block, obj.R);
            end
            obj.H = blkdiag(Q_block, obj.P, R_block);
        end
        
        function construct_eq_constraint(obj)
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
            obj.C_neq1 = [F_block, [G_block; zeros(obj.nc, obj.nu*obj.N)]];
            obj.nc_total = size(obj.C_neq1)*[1; 0];
            obj.C_neq2 = ones(obj.nc_total, 1);
        end
        
        function compute_MPIset(obj)
            % MPIset is computed only once in the constructor;
            [F, G, obj.nc] = obj.convert_Poly2Mat(obj.Xc, obj.Uc);
            Fpi = @(i) (F+G*obj.K)*obj.Ak^i;
            Xpi = @(i) Polyhedron(Fpi(i), ones(size(Fpi(i))*[1; 0], 1));
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
    
    %% Constraint Manipulation 
    methods (Access = protected)
        
        function add_ineq_constraint(obj, Xadd, k_add)
            % add a new constraint at time step k 
            if Xadd.contains(zeros(2, 1)) % If Xadd contains the origin, the contraint can be expressed as C1*x<=1
                [F_add, ~, nc_add] = obj.convert_Poly2Mat(Xadd, Polyhedron());
                obj.C_neq2 = [obj.C_neq2; ones(nc_add, 1)];
                
            else % in other cases, expressed in a general affine form C1*x<=C2
                F_add = Xadd.A;
                nc_add = size(F_add)*[1; 0];
                obj.C_neq2 = [obj.C_neq2; Xadd.b];
            end
            
            block_add = zeros(nc_add, obj.n_opt);
            block_add(:, (k_add-1)*obj.nx+1:k_add*obj.nx) = F_add;
            
            obj.C_neq1 = [obj.C_neq1;
                block_add];
            obj.nc = obj.nc + nc_add;
        end
        
        function [F, G, nc] = convert_Poly2Mat(obj, X, U)
            % Constraints set X and U in Polyhedron form -> F, G matrix form
            %F; G; % constraints for state and input: Fx+Gu<=1, where 1 is a voctor
            F_tmp = OptimalControlBasis.poly2ineq(X);
            G_tmp = OptimalControlBasis.poly2ineq(U);
            if numel(F_tmp)==0
                F_tmp = zeros(0, obj.nx);
            end
            if numel(G_tmp)==0
                G_tmp = zeros(0, obj.nu);
            end
            F = [F_tmp; zeros(size(G_tmp)*[1; 0], obj.nx)];
            G = [zeros(size(F_tmp)*[1; 0], obj.nu); G_tmp];
            nc = size(F)*[1; 0];
        end
        

        
    end
    
    %% Static Methods
    methods(Static)
        
        function Cneq = poly2ineq(poly)
            Cneq  = poly.A./repmat(poly.b, 1, size(poly.A)*[0; 1]);
        end
        
                
    end
end