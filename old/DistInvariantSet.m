classdef DistInvariantSet < Polyhedron & GraphicsBasis
    methods (Access = public)
        
        function obj = DistInvariantSet(Ak, W, varargin)
            % Ak (=A+BK) is dynamics of automonous system 
            % W is a convex disturbance set
            alpha = 1.1;
            Z = (W+Ak*W+Ak^2*W+Ak^3*W)*alpha;
            Z.minHRep();
            Z.minVRep();
            obj@Polyhedron('V', Z.V);
            obj@GraphicsBasis();
        end
        
        function show(obj)
            obj.show_convex(Polyhedron('V', obj.V), [0.6, 0.6, 0.6]);
        end
        
    end
    
    methods (Access = private)
        function calc_SimpleApproximation(obj, Ak, W)
            
        end
        
        function calc_FancyApproximation(obj, Ak, W)
            % Implementation of Rakovic et al.: Invariant Approximation of
            % the Minimal Robust Positively Invariant Set, IEEE Trans. on
            % Auto. Cont., 2005.
            
            % will-be-added
        end
    end
    
end