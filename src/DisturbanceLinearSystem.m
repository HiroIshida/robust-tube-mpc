classdef DisturbanceLinearSystem < LinearSystem

    properties (SetAccess = private)
        W % convex set of distrubance
        Z % disturbance invariant set
    end

    methods (Access = public)

        function obj = DisturbanceLinearSystem(A, B, Q, R, W)
            obj = obj@LinearSystem(A, B, Q, R);

            obj.W = W;
            obj.Z = obj.compute_mrpi_set(1e-4);
        end

        function x_new = propagate(obj, x, u)
            w = obj.pick_random_disturbance();
            x_new = propagate@LinearSystem(obj, x, u) + w;
        end

    end

    methods (Access = public)

        function w = pick_random_disturbance(obj)
            % pick disturbance form uniform distribution
            verts = obj.W.V;
            b_max = max(verts)';
            b_min = min(verts)';

            % generate random until it will be inside of W
            while true
                w = rand(obj.nx, 1) .* (b_max - b_min) + b_min; 
                if obj.W.contains(w)
                    break
                end
            end
        end

        function Fs = compute_mrpi_set(obj, epsilon)
            % Computes an invariant approximation of the minimal robust positively
            % invariant set for 
            % x^{+} = Ax + w with w \in W
            % according to Algorithm 1 in 'Invariant approximations of
            % the minimal robust positively invariant set' by Rakovic et al. 
            % Requires a matrix A, a Polytope W, and a tolerance 'epsilon'.  
            [nx,~] = size(obj.Ak); 
            s = 0; 
            alpha = 1000;
            Ms = 1000;
            E = eye(nx);
            it = 0;
            while(alpha > epsilon/(epsilon + Ms))
                s = s+1;
                alpha = max(obj.W.support(obj.Ak^s*(obj.W.A)')./obj.W.b);
                mss = zeros(2*nx,1);
                for i = 1:s
                    mss = mss+obj.W.support([obj.Ak^i, -obj.Ak^i]);
                end
                Ms = max(mss);
                it = it+1;
            end

            Fs = obj.W;
            for i =1:s-1
                Fs = Fs+obj.Ak^i*obj.W;
            end
            Fs = (1/(1-alpha))*Fs;
        end

    end
end
