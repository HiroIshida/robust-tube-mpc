classdef MPC_Constraint < Constraint
    properties (SetAccess = protected)
        A; B;
        Q; R;
        K; 
        P;
        Ak;
        N;
        
        n_opt;
        nc_total;
        
        H;
        C_eq1; C_eq2;
        C_neq1; C_neq2;
    end
    
    methods (Access = public)
        function obj = MPC_Constraint(A, B, Q, R, Xc, Uc, N)
            obj@Constraint(Xc, Uc, size(A)*[1; 0], size(B)*[0; 1]);
            obj.A = A;
            obj.B = B;
            obj.Q = Q;
            obj.R = R;
            obj.N = N;
            
            [K_tmp, obj.P] = dlqr(obj.A, obj.B, obj.Q, obj.R);
            obj.K = -K_tmp;
            obj.Ak = (obj.A+obj.B*obj.K);
            obj.n_opt = obj.nx*(obj.N+1)+obj.nu*obj.N;
            
            
            obj.construct_MPC_costfunction();
            obj.construct_MPC_eq_constraint();
            %obj.construct_MPC_ineq_constraint();
        end
    end
    
    methods (Access = protected)
        
        function construct_MPC_costfunction(obj)
            % compute H
            Q_block = [];
            R_block = [];
            for itr=1:obj.N
                Q_block = blkdiag(Q_block, obj.Q);
                R_block = blkdiag(R_block, obj.R);
            end
            obj.H = blkdiag(Q_block, obj.P, R_block);
        end
        
        function construct_MPC_eq_constraint(obj)
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
        
        function construct_MPC_ineq_constraint(obj)
            % compute C_neq
            F_block = [];
            G_block = [];
            for itr = 1:obj.N
                G_block = blkdiag(G_block, obj.G);
            end
            for itr = 1:obj.N+1
                F_block = blkdiag(F_block, obj.F);
            end
            obj.C_neq1 = [F_block, [G_block; zeros(obj.nc, obj.nu*obj.N)]];
            obj.nc_total = size(obj.C_neq1)*[1; 0];
            obj.C_neq2 = zeros(obj.nc_total, 1);
        end
        
        function add_terminal_constraint(obj, Xadd)
            add_ineq_constraint(obj, Xadd, [], obj.N+1);
        end
        
        function add_initial_constraint(obj, Xadd, Uadd)
            add_ineq_constraint(obj, Xadd, Uadd, 1);
        end
        
        function add_ineq_constraint(obj, Xadd, Uadd, k_add)
            % add a new constraint at time step k 
            
            C_add = Constraint(Xadd, Uadd);
            F_add = C_add.F;
            G_add = C_add.G;
            nc_add = C_add.nc;
            
            block_x_add = zeros(nc_add, obj.nx*(obj.N+1));
            block_x_add(:, obj.nx*(k_add-1)+1:obj.nx*k_add) = F_add;
            
            if k_add<obj.N+1
                block_u_add = zeros(nc_add, obj.nu*obj.N);
                block_u_add(:, obj.nu*(k_add-1)+1:obj.nu*(k_add-1)+obj.nu) = G_add; 
            else
                block_u_add = zeros(nc_add, obj.nu*obj.N);
            end
            
            block_add = [block_x_add, block_u_add];
            obj.C_neq1 = [obj.Cneq1;
                block_add];
            obj.C_neq2 = zeros(obj.nc_total, 1);

            obj.nc_total = obj.nc_total + nc_add;
        end
    end
end