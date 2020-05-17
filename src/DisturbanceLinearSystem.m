classdef DisturbanceLinearSystem < LinearSystem

    properties (SetAccess = private)
        W % convex set of distrubance
    end

    methods (Access = public)

        function obj = DisturbanceLinearSystem(A, B, Q, R, W)
            obj = obj@LinearSystem(A, B, Q, R);
            obj.W = W;
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
    end
end
