classdef DisturbanceLinearSystem < LinearSystem

    properties (SetAccess = private)
        W % convex set of distrubance
        Z % disturbance invariant set
    end

    methods (Access = public)

        function obj = DisturbanceLinearSystem(A, B, Q, R, W)
            obj = obj@LinearSystem(A, B, Q, R);

            obj.W = W;
            obj.Z = obj.compute_distinv_set(3, 1.05)
        end

        function x_new = propagate(obj, x, u)
            w = obj.pick_random_disturbance();
            x_new = propagate@LinearSystem(obj, x, u) + w;
        end


    end

    methods (Access = public)

        function w = pick_random_disturbance(obj)
            verts = obj.W.V;
            b_max = max(verts)';
            b_min = min(verts)';
            w = rand(obj.nx, 1) .* (b_max - b_min) + b_min; 
        end

        function Z_approx = compute_distinv_set(obj, n_order, alpha)
            % W: Polyhedron of system noise
            % We could obtain dist_inv_set Z by computing an infinite geometric series,
            %  which is not practicall to get. So, we approximate this by trancating the polynomial.
            Z_approx = obj.W;
            for n = 1:n_order
                Z_approx = Z_approx + obj.Ak^n*obj.W;
            end
            Z_approx = Z_approx*alpha;
            % which takes the form of Z = alpha*(W + Ak*W + Ak^2*W + ... Ak^n_ordr*W).
            % where + denotes Minkowski addition.
        end

    end
end
