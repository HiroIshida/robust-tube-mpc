classdef tube_mpc < mpc
    properties (Access = public)
        W; % disturbance set (polytope)
        Z; % disturbance invariant set (polytope)
        Xter; % terminal constraint (set to MPI set)
        Xter_robust; % robust terminal constraint (X_ter - Z)
        Xc_robust;
        Uc_robust;
        F_robust;
        G_robust;
        nc_robust;
        
        k_time;
    end
    methods (Access = public)
        function obj = tube_mpc(A_init, B_init, Q_init, R_init, F_init, G_init, N_init, W_init)
            obj@mpc(A_init, B_init, Q_init, R_init, F_init, G_init, N_init)
            obj.W = W_init;
            obj.disturbance(W_init); % if you want to make a change of W_real from W, re-set the disturbance by "obj.dusturbance(W_your)"
            obj.Z = (obj.W+obj.Ak*obj.W+obj.Ak^2*obj.W+obj.Ak^3*obj.W)*1.05; % tempolary
            obj.Z.minHRep;

            % modify inequality constraints            
            obj.Xc_robust = obj.Xc - obj.Z;
            obj.Uc_robust = obj.Uc - obj.K*obj.Z;
            obj.Xc_robust.minHRep;
            obj.Uc_robust.minHRep;
            obj.Uc_robust
            
            obj.nc_robust = size(obj.Xc_robust.A)*[1; 0]+size(obj.Uc_robust.A)*[1; 0];
            
            F_robust_tmp = obj.Xc_robust.A./repmat(obj.Xc_robust.b, 1, obj.nx);
            G_robust_tmp =  obj.Uc_robust.A./repmat(obj.Uc_robust.b, 1, obj.nu);
            obj.F_robust = [F_robust_tmp;
                zeros(size(obj.Uc_robust.A)*[1; 0], obj.nx)];
            obj.G_robust = [zeros(size(obj.Xc_robust.A)*[1; 0], obj.nu);
                G_robust_tmp];
            
            F_block = obj.F; % NOT obj.F_robust here
            G_block = obj.G; % NOT obj.G_robust here
            for itr = 2:obj.N
                G_block = blkdiag(G_block, obj.G_robust);
            end
            for itr = 2:obj.N+1
                F_block = blkdiag(F_block, obj.F_robust);
            end
            size(obj.F_robust)
            size(obj.G_robust)
            obj.C_neq = [F_block, [G_block; zeros(obj.nc_robust, obj.nu*obj.N)]];
            
            % compute MPI set. (approximately)
            FGK = obj.F + obj.G*obj.K;
            Xpi = @(i) Polyhedron(FGK*obj.Ak^i, ones(obj.nc, 1));
            Xmpi = Xpi(0);
            for i=1:5
                Xmpi = tube_mpc.set_intersection(Xmpi, Xpi(i));
            end
            Xmpi.minHRep;
            obj.Xter = Xmpi;
            
            % robust MPI set
            obj.Xter_robust = obj.Xter - obj.Z;
            obj.Xter_robust.minHRep;
            
            % add the termnal constraint
            F_ter_robust = obj.Xter_robust.A./repmat(obj.Xter_robust.b, 1, obj.nx);
            dim_F_ter_robust = size(F_ter_robust)*[1; 0];
            obj.C_neq = [obj.C_neq;
                zeros(dim_F_ter_robust, obj.nx*obj.N), F_ter_robust, zeros(dim_F_ter_robust, obj.nu*obj.N)];
            
        end
        function input = solve(obj)
            x = obj.x_current;
            if obj.flag_init == 0
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
                obj.x_seq_nominal_init = obj.x_seq_nominal;
                obj.u_seq_nominal_init = obj.u_seq_nominal;
                obj.flag_init = 1;
                input = obj.u_seq_nominal(1,1);
                obj.k_time = 2;
            else
                if obj.k_time<obj.N+1
                    input = obj.u_seq_nominal_init(:, obj.k_time) + obj.K*(x-obj.x_seq_nominal_init(:, obj.k_time));
                else
                    input = obj.K*x;
                end
                obj.k_time = obj.k_time+1;
            end
        end
            
        function [] = show_Xc_robust(obj)
            obj.show_constraint(obj.Xc_robust, 'r');
        end
        function [] = show_tube(obj)
            for k=1:obj.N+1
                obj.show_constraint((obj.x_seq_nominal_init(:, k) + obj.Z), 'g', 'FaceAlpha', .2);
                hold on;
            end
        end
        function [] = show_Xter(obj)
            obj.show_constraint(obj.Xter, [0.3, 0.3, 0.3]);
        end
        function [] = show_Xter_robust(obj)
            obj.show_constraint(obj.Xter_robust, 'w');
        end
        function [] = show_Z(obj)
            obj.show_constraint(obj.Z, 'w');
        end
                
    end
    methods (Static)
        function S12 = set_intersection(S1, S2)
            S12 = Polyhedron([S1.A; S2.A], [S1.b; S2.b]);
            S12.minHRep;
        end
    end
    %{
    methods (Static)
        % c.f.) RAKOVIC, Sasa V., et al. Invariant approximations of the minimal robust positively invariant set.
        % IEEE Transactions on Automatic Control, 2005, 50.3: 406-410.
        
        function Z_aprox = calc_dinv_set(Ak, W, eps)
            f = W.A'; 
            g = W.b;
            hw = @(a, f, g) a'*linprog(a, f, g);
            s = 0;
            while(1)
                s = s+1;
                alpha_optim = tube_mpc.calc_alpha_optim(s, hw, Ak, f, g);
                alpha = alpha_optim;
                M = tube_mpc.calc_M(s, hw, Ak, f, g);
                if(alpha<=eps/(eps+M))
                    break
                end
            end
            Fs = Polyhedron;
            s
            alpha
            for i=0:s-1
                Fs = Fs + Ak^i*W;
            end
            Z_aprox = Fs*(1/(1-alpha));
        end
                     
        function alpha_optim = calc_alpha_optim(s, hw, Ak, f, g)
            % compute Eq. 11 in the reference.
            aux = @(idx) hw((Ak^s)'*f(:, idx), f(:, idx)', g(idx))/g(idx);
            
            val_max = aux(1);       
            for itr=2:numel(g)
                if aux(itr)>val_max
                    val_max = aux(itr);
                end
            end
            alpha_optim  =val_max;
        end
        
        function M = calc_M(W, Ak, s)
            F = tube_mpc.Fs(W, Ak, s);
            vtx = (F.V)';
            lower = 0.1;
            upper = 100;
            cost = 100;
            size(vtx)
            while(or(cost<=-0.001, cost>0))
                mid = (lower+upper)*0.5;
                Ball = Polyhedron([1, 1; 1, -1; -1, -1; -1, 1]*mid);
                cost = max(max(-Ball.A*vtx+repmat(Ball.b, 1, size(vtx)*[0; 1])));
                if cost>0
                    upper = mid;
                else
                    lower = mid;
                end    
                mid
            end
            M = mid;
        end
            
        
        function F = Fs(W, Ak, s)
            F = Polyhedron;
            for i = 0:s-1
                F = F + Ak^i*W;
            end
        end
        %{
        function M = calc_M(s, hw, Ak, f, g)
            % compute Eq. 13 in the reference
            nx = size(Ak)*[1; 0];
            e_zeros = zeros(nx, 1);
            max_val = -1;
            for j=1:nx
                ej = e_zeros;
                ej(j) = 1;
                left = 0;
                right = 0;
                size((Ak^2)'*ej)
                size(f')
                size(g)
                for i=0:s-1
                    left = left + hw((Ak^i)'*ej, f', g);
                    right = right + hw(-(Ak^i)'*ej, f', g);
                end
                max_val_tmp = max(left, right);
                if max_val_tmp>max_val
                    max_val = max_val_tmp;
                end
            end
            M = max_val;
        end
        %}
        
    end
    %}
            
        
end